"""Automated sensor validation run for the Husky.

Drives a fixed test while recording every sensor and the simulator's ground truth:

    settle -> drive straight -> stop -> turn in place -> stop -> drive straight -> stop

The second straight turns any heading error into a visible position error. The node then saves CSV files, a text summary and a 6-panel figure. Every sensor is compared
against ground truth (Gazebo's OdometryPublisher), and all timestamps are simulation time.
"""
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan


# ============================================================ maths helpers

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def ray_box(ox, oy, angles, box):
    """Distance along 2D rays from (ox, oy) at `angles` to an axis-aligned box (inf on a miss)."""
    xmin, xmax, ymin, ymax = box
    dx, dy = np.cos(angles), np.sin(angles)
    with np.errstate(divide='ignore', invalid='ignore'):
        tx1, tx2 = (xmin - ox) / dx, (xmax - ox) / dx
        ty1, ty2 = (ymin - oy) / dy, (ymax - oy) / dy
        t_near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        t_far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        hit = (t_far >= 0.0) & (t_near <= t_far)
    return np.where(hit, np.where(t_near >= 0.0, t_near, t_far), np.inf)


def integrate(t, y):
    """Cumulative trapezoidal integral, starting at 0."""
    if len(t) < 2:
        return np.zeros_like(t)
    return np.concatenate(([0.0], np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))))


def path_distance(x, y, yaw):
    """Signed distance travelled along the heading (negative when reversing)."""
    d = np.zeros_like(x)
    d[1:] = np.cumsum(np.diff(x) * np.cos(yaw[:-1]) + np.diff(y) * np.sin(yaw[:-1]))
    return d


def at(tq, t, y):
    return float(np.interp(tq, t, y))


def default_output_dir(prefix=None):
    """husky_validation/ next to the colcon workspace: <ws>/install/... -> <ws>/../husky_validation."""
    try:
        path = prefix or get_package_prefix('husky_sensor_sim')
    except Exception:
        path = ''
    while path and path != os.path.dirname(path) and os.path.basename(path) != 'install':
        path = os.path.dirname(path)
    if os.path.basename(path) == 'install':
        workspace = os.path.dirname(path)
        return os.path.join(os.path.dirname(workspace), 'husky_validation')
    return os.path.expanduser('~/husky_validation')


def rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else float('nan')


# ============================================================ analysis

def analyze(data, cfg):
    """Turn the recorded streams into aligned series and validation metrics."""
    events = data['events']
    t0 = events[0][0]
    phase_t = {}
    for t, name in events:
        phase_t.setdefault(name, t - t0)
    te = phase_t['done']
    ps = phase_t.get('straight', te)
    p_stop1 = phase_t.get('stop1', te)
    pt = phase_t.get('turn', te)
    p_stop2 = phase_t.get('stop2', te)
    p_s2 = phase_t.get('straight2', te)   # end of the turn measurement window
    p_stop3 = phase_t.get('stop3', te)

    def crop(rows):
        a = np.array(rows, dtype=float)
        a[:, 0] -= t0
        return a[(a[:, 0] >= 0.0) & (a[:, 0] <= te)]

    r = {'cfg': cfg, 'phase_t': phase_t,
         'spans': {'straight': (ps, p_stop1), 'turn': (pt, p_stop2), 'straight2': (p_s2, p_stop3)}}

    # --- ground truth
    gt = crop(data['gt'])
    g_t, g_x, g_y = gt[:, 0], gt[:, 1], gt[:, 2]
    g_yaw = np.unwrap(gt[:, 3])
    r.update(g_t=g_t, g_x=g_x, g_y=g_y, g_v=gt[:, 4], g_w=gt[:, 5],
             g_yaw_rel=g_yaw - g_yaw[0], g_dist=path_distance(g_x, g_y, g_yaw))
    gx0, gy0, gyaw0 = g_x[0], g_y[0], g_yaw[0]

    def odometry_series(rows):
        """Odometry relative to its start, placed in the world at the true start pose."""
        od = crop(rows)
        yaw = np.unwrap(od[:, 3])
        dx, dy = od[:, 1] - od[0, 1], od[:, 2] - od[0, 2]
        c0, s0 = math.cos(gyaw0 - yaw[0]), math.sin(gyaw0 - yaw[0])
        return (od[:, 0], yaw - yaw[0], path_distance(od[:, 1], od[:, 2], yaw),
                gx0 + c0 * dx - s0 * dy, gy0 + s0 * dx + c0 * dy)

    # --- DiffDrive odometry (computed by Gazebo from the wheel joints)
    o_t, o_yaw_rel, o_dist, o_xw, o_yw = odometry_series(data['odom'])
    r.update(o_t=o_t, o_yaw_rel=o_yaw_rel, o_dist=o_dist, o_xw=o_xw, o_yw=o_yw)

    # --- EKF (robot_localization: encoder speed + IMU heading), optional
    r['has_ekf'] = len(data.get('ekf') or []) > 1
    if r['has_ekf']:
        f_t, f_yaw_rel, f_dist, f_xw, f_yw = odometry_series(data['ekf'])
        r.update(f_t=f_t, f_yaw_rel=f_yaw_rel, f_dist=f_dist, f_xw=f_xw, f_yw=f_yw)

    # --- IMU
    im = crop(data['imu'])
    i_t, i_wz = im[:, 0], im[:, 2]
    i_yaw = np.unwrap(im[:, 1])
    r.update(i_t=i_t, i_wz=i_wz, i_az=im[:, 5], i_yaw_rel=i_yaw - i_yaw[0],
             i_gyro_yaw=integrate(i_t, i_wz))

    # --- wheel encoders
    names = data['enc_names']
    k = len(names)
    en = crop(data['enc'])
    e_t, pos, vel = en[:, 0], en[:, 1:1 + k], en[:, 1 + k:1 + 2 * k]
    left = [i for i, n in enumerate(names) if 'left' in n]
    right = [i for i, n in enumerate(names) if 'right' in n]
    rad, track = cfg['wheel_radius'], cfg['track']
    L = pos[:, left].mean(axis=1)
    R = pos[:, right].mean(axis=1)
    L, R = L - L[0], R - R[0]
    vl, vr = vel[:, left].mean(axis=1) * rad, vel[:, right].mean(axis=1) * rad
    e_dist = rad * (L + R) / 2.0
    r.update(e_t=e_t, e_names=names, e_wheel_vel=vel, e_dist=e_dist,
             e_yaw=rad * (R - L) / track, e_v=(vl + vr) / 2.0, e_w=(vr - vl) / track)

    # --- dead reckoning: encoder distance + IMU heading
    yaw_imu_at_e = np.interp(e_t, i_t, r['i_yaw_rel']) + gyaw0
    ds = np.diff(e_dist)
    r['dr_x'] = gx0 + np.concatenate(([0.0], np.cumsum(ds * np.cos(yaw_imu_at_e[:-1]))))
    r['dr_y'] = gy0 + np.concatenate(([0.0], np.cumsum(ds * np.sin(yaw_imu_at_e[:-1]))))

    # --- lidar: expected range of every beam from the true pose and the known box
    meta, box = data['scan_meta'], cfg['box']
    s_t_all = np.array(data['scan_t']) - t0
    keep = (s_t_all >= 0.0) & (s_t_all <= te)
    s_t = s_t_all[keep]
    scans = [s for s, kp in zip(data['scan_ranges'], keep) if kp]
    beam_offsets = meta['angle_min'] + meta['angle_increment'] * np.arange(len(scans[0]) if scans else 0)
    i0 = int(round(-meta['angle_min'] / meta['angle_increment']))
    rmin, rmax = meta['range_min'], meta['range_max']
    fwd_meas, fwd_exp, scan_mae, errors = [], [], [], []
    missed = unexpected = 0
    for ts, ranges in zip(s_t, scans):
        x, y, yaw = at(ts, g_t, g_x), at(ts, g_t, g_y), at(ts, g_t, g_yaw)
        expected = ray_box(x, y, yaw + beam_offsets, box)
        meas_ok = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
        exp_ok = (expected >= rmin) & (expected <= rmax)
        both = meas_ok & exp_ok
        err = ranges[both] - expected[both]
        errors.append(err)
        scan_mae.append(np.mean(np.abs(err)) if err.size else np.nan)
        missed += int(np.sum(exp_ok & ~meas_ok))
        unexpected += int(np.sum(meas_ok & ~exp_ok))
        fwd_meas.append(ranges[i0] if meas_ok[i0] else np.nan)
        fwd_exp.append(expected[i0] if exp_ok[i0] else np.nan)
    all_err = np.concatenate(errors) if errors else np.array([])
    fwd_meas, fwd_exp = np.array(fwd_meas), np.array(fwd_exp)
    fwd_err = fwd_meas - fwd_exp
    fwd_err = fwd_err[np.isfinite(fwd_err)]
    r.update(s_t=s_t, fwd_meas=fwd_meas, fwd_exp=fwd_exp, scan_mae=np.array(scan_mae))

    # --- metrics
    m = {}
    m['sim_time'] = te
    wall = data['wall']
    m['rtf'] = te / (wall[1] - wall[0]) if wall[1] > wall[0] else float('nan')

    def delta(t, y, a, b):
        return at(b, t, y) - at(a, t, y)

    m['straight'] = {
        'truth': delta(g_t, r['g_dist'], ps, pt),
        'encoders': delta(e_t, e_dist, ps, pt),
        'odom': delta(o_t, r['o_dist'], ps, pt),
    }
    if r['has_ekf']:
        m['straight']['ekf'] = delta(r['f_t'], r['f_dist'], ps, pt)
    sel = (e_t >= ps) & (e_t <= p_stop1)
    m['speed_rms_err'] = rms(r['e_v'][sel] - np.interp(e_t[sel], g_t, r['g_v']))
    m['speed_mean'] = float(np.mean(r['e_v'][sel])) if sel.any() else float('nan')

    m['turn'] = {
        'truth': math.degrees(delta(g_t, r['g_yaw_rel'], pt, p_s2)),
        'imu_gyro': math.degrees(delta(i_t, r['i_gyro_yaw'], pt, p_s2)),
        'imu_orientation': math.degrees(delta(i_t, r['i_yaw_rel'], pt, p_s2)),
        'encoders': math.degrees(delta(e_t, r['e_yaw'], pt, p_s2)),
        'odom': math.degrees(delta(o_t, r['o_yaw_rel'], pt, p_s2)),
    }
    if r['has_ekf']:
        m['turn']['ekf'] = math.degrees(delta(r['f_t'], r['f_yaw_rel'], pt, p_s2))
    gt_turn = m['turn']['truth']
    m['effective_track'] = (track * m['turn']['encoders'] / gt_turn) if abs(gt_turn) > 1.0 else float('nan')

    gxe, gye = g_x[-1], g_y[-1]
    m['final_err_odom'] = math.hypot(r['o_xw'][-1] - gxe, r['o_yw'][-1] - gye)
    m['final_err_dr'] = math.hypot(r['dr_x'][-1] - gxe, r['dr_y'][-1] - gye)
    if r['has_ekf']:
        m['final_err_ekf'] = math.hypot(r['f_xw'][-1] - gxe, r['f_yw'][-1] - gye)

    rest = i_t <= ps
    m['rest_az_mean'] = float(np.mean(r['i_az'][rest])) if rest.any() else float('nan')
    m['rest_az_std'] = float(np.std(r['i_az'][rest])) if rest.any() else float('nan')
    m['rest_gyro_bias'] = float(np.mean(i_wz[rest])) if rest.any() else float('nan')

    m['lidar'] = {
        'scans': len(s_t), 'hits': int(all_err.size), 'missed': missed, 'unexpected': unexpected,
        'bias': float(np.mean(all_err)) if all_err.size else float('nan'),
        'mae': float(np.mean(np.abs(all_err))) if all_err.size else float('nan'),
        'rms': rms(all_err),
        'max': float(np.max(np.abs(all_err))) if all_err.size else float('nan'),
        'fwd_mae': float(np.mean(np.abs(fwd_err))) if fwd_err.size else float('nan'),
    }
    r['metrics'] = m
    r['summary'] = summary_text(m, cfg)
    return r


def summary_text(m, cfg):
    s, t, ld = m['straight'], m['turn'], m['lidar']

    def pct(err, ref):
        return f'{100.0 * err / ref:+.2f} %' if abs(ref) > 1e-6 else ''

    has_ekf = 'ekf' in s
    lines = [
        '=== HUSKY SENSOR VALIDATION ===',
        f"Sim time {m['sim_time']:.1f} s   real-time factor {m['rtf']:.2f}",
        '',
        f"STRAIGHT   truth {s['truth']:.3f} m",
        f"  encoders  {s['encoders']:7.3f} m  err {s['encoders'] - s['truth']:+.3f} m"
        f"  {pct(s['encoders'] - s['truth'], s['truth'])}",
        f"  /odom     {s['odom']:7.3f} m  err {s['odom'] - s['truth']:+.3f} m",
    ]
    if has_ekf:
        lines.append(f"  EKF       {s['ekf']:7.3f} m  err {s['ekf'] - s['truth']:+.3f} m")
    lines += [
        f"  speed {m['speed_mean']:.3f} m/s, RMS err vs truth {m['speed_rms_err']:.4f} m/s",
        '',
        f"TURN       truth {t['truth']:.1f} deg",
        f"  IMU gyro  {t['imu_gyro']:7.1f} deg  err {t['imu_gyro'] - t['truth']:+.2f}",
        f"  IMU orien {t['imu_orientation']:7.1f} deg  err {t['imu_orientation'] - t['truth']:+.2f}",
        f"  encoders  {t['encoders']:7.1f} deg  err {t['encoders'] - t['truth']:+.2f}  (skid slip)",
        f"  /odom     {t['odom']:7.1f} deg  err {t['odom'] - t['truth']:+.2f}",
    ]
    if has_ekf:
        lines.append(f"  EKF       {t['ekf']:7.1f} deg  err {t['ekf'] - t['truth']:+.2f}")
    lines += [
        f"  effective track {m['effective_track']:.3f} m (nominal {cfg['track']:.3f} m)",
        '',
        'FINAL POSITION ERROR vs truth',
        f"  /odom (encoders only)  {m['final_err_odom']:.3f} m",
        f"  encoders + IMU heading {m['final_err_dr']:.3f} m",
    ]
    if has_ekf:
        lines.append(f"  EKF (encoders + IMU)   {m['final_err_ekf']:.3f} m")
    else:
        lines.append('  EKF: no /odometry/filtered recorded')
    lines += [
        '',
        f"LIDAR  {ld['scans']} scans, {ld['hits']} beam hits on obstacle",
        f"  bias {ld['bias']:+.4f} m   mean|err| {ld['mae']:.4f} m",
        f"  RMS {ld['rms']:.4f} m   max {ld['max']:.4f} m",
        f"  forward beam mean|err| {ld['fwd_mae']:.4f} m",
        f"  missed {ld['missed']}, unexpected {ld['unexpected']} (box edges)",
        '',
        'IMU AT REST',
        f"  accel z {m['rest_az_mean']:.3f} +/- {m['rest_az_std']:.3f} m/s^2 (gravity 9.8)",
        f"  gyro z bias {m['rest_gyro_bias']:+.5f} rad/s",
    ]
    return '\n'.join(lines)


# ============================================================ output

def save_csv(data, r, out_dir):
    def write(name, rows, header):
        np.savetxt(os.path.join(out_dir, name), np.asarray(rows, dtype=float),
                   delimiter=',', header=header, comments='', fmt='%.6f')

    t0 = data['events'][0][0]
    shift = lambda rows: [[row[0] - t0, *row[1:]] for row in rows]  # noqa: E731
    write('ground_truth.csv', shift(data['gt']), 't,x,y,yaw,v,w')
    write('odom.csv', shift(data['odom']), 't,x,y,yaw')
    if data.get('ekf'):
        write('ekf.csv', shift(data['ekf']), 't,x,y,yaw')
    write('imu.csv', shift(data['imu']), 't,yaw,gyro_z,accel_x,accel_y,accel_z')
    names = data['enc_names']
    write('encoders.csv', shift(data['enc']),
          't,' + ','.join(f'{n}_pos' for n in names) + ',' + ','.join(f'{n}_vel' for n in names))
    write('lidar.csv', np.column_stack([r['s_t'], r['fwd_meas'], r['fwd_exp'], r['scan_mae']]),
          't,forward_measured,forward_expected,obstacle_beams_mean_abs_err')
    with open(os.path.join(out_dir, 'phases.csv'), 'w') as f:
        f.write('t,phase\n')
        for t, name in data['events']:
            f.write(f'{t - t0:.6f},{name}\n')
    with open(os.path.join(out_dir, 'summary.txt'), 'w') as f:
        f.write(r['summary'] + '\n')


def make_figure(r, path, summary_path, show):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig = plt.figure(figsize=(15, 15))
    grid = fig.add_gridspec(3, 2)
    axes = {'heading': fig.add_subplot(grid[0, 0]), 'lidar': fig.add_subplot(grid[0, 1]),
            'wheels': fig.add_subplot(grid[1, 0]), 'velocity': fig.add_subplot(grid[1, 1]),
            'path': fig.add_subplot(grid[2, :])}

    def shade(ax):
        for name, color in (('straight', 'tab:green'), ('turn', 'tab:blue'), ('straight2', 'tab:green')):
            a, b = r['spans'][name]
            if b > a:
                ax.axvspan(a, b, color=color, alpha=0.08)

    # 1. heading
    ax = axes['heading']
    ax.plot(r['g_t'], np.degrees(r['g_yaw_rel']), 'k-', lw=2.5, label='Ground truth')
    ax.plot(r['i_t'], np.degrees(r['i_yaw_rel']), 'b-', lw=1.5, label='IMU orientation')
    ax.plot(r['i_t'], np.degrees(r['i_gyro_yaw']), 'c--', lw=1.5, label='IMU gyro integrated')
    ax.plot(r['e_t'], np.degrees(r['e_yaw']), 'r-', lw=1.5, label='Encoders')
    ax.plot(r['o_t'], np.degrees(r['o_yaw_rel']), color='orange', ls=':', lw=2, label='/odom')
    if r['has_ekf']:
        ax.plot(r['f_t'], np.degrees(r['f_yaw_rel']), color='tab:purple', ls='-.', lw=2,
                label='EKF (encoders + IMU)')
    shade(ax)
    ax.set_title('Heading change vs ground truth')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (deg)')
    ax.legend()
    ax.grid(True)

    # 2. lidar
    ax = axes['lidar']
    ax.plot(r['s_t'], r['fwd_meas'], 'b-', lw=2, label='Forward beam (measured)')
    ax.plot(r['s_t'], r['fwd_exp'], 'r--', lw=2, label='Forward beam (expected from truth)')
    shade(ax)
    ax.set_title('Lidar range to obstacle')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range (m)')
    ax.grid(True)
    ax2 = ax.twinx()
    ax2.plot(r['s_t'], r['scan_mae'] * 100.0, color='tab:orange', lw=1, alpha=0.8,
             label='All obstacle beams mean |err|')
    ax2.set_ylabel('Mean |error| (cm)', color='tab:orange')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper right')

    # 3. wheel velocities
    ax = axes['wheels']
    colors = {'left': 'tab:blue', 'right': 'tab:red'}
    for i, name in enumerate(r['e_names']):
        side = 'left' if 'left' in name else 'right'
        ax.plot(r['e_t'], r['e_wheel_vel'][:, i], color=colors[side], lw=1.5,
                ls='--' if 'rear' in name else '-', label=name.replace('_wheel_joint', ''))
    shade(ax)
    ax.axhline(0.0, color='k', lw=0.5)
    ax.set_title('Wheel encoders: angular velocity')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('rad/s')
    ax.legend()
    ax.grid(True)

    # 4. robot velocity
    ax = axes['velocity']
    ax.plot(r['e_t'], r['e_v'], 'g-', lw=2, label='Linear, encoders (m/s)')
    ax.plot(r['g_t'], r['g_v'], 'k--', lw=1.2, label='Linear, truth (m/s)')
    ax.plot(r['e_t'], r['e_w'], 'm-', lw=1.5, label='Angular, encoders (rad/s)')
    ax.plot(r['i_t'], r['i_wz'], 'c-', lw=0.8, alpha=0.6, label='Angular, IMU gyro (rad/s)')
    ax.plot(r['g_t'], r['g_w'], 'k:', lw=1.5, label='Angular, truth (rad/s)')
    shade(ax)
    ax.set_title('Robot velocity: encoders and IMU vs truth')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.legend()
    ax.grid(True)

    # 5. path (full width)
    ax = axes['path']
    xmin, xmax, ymin, ymax = r['cfg']['box']
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, color='tab:red', alpha=0.4,
                           label='Obstacle'))
    ax.plot(r['g_x'], r['g_y'], 'k-', lw=2.5, label='Ground truth')
    ax.plot(r['o_xw'], r['o_yw'], color='orange', ls=':', lw=2, label='/odom (encoders only)')
    ax.plot(r['dr_x'], r['dr_y'], 'b--', lw=1.5, label='Encoders + IMU heading')
    if r['has_ekf']:
        ax.plot(r['f_xw'], r['f_yw'], color='tab:purple', ls='-.', lw=2, label='EKF (encoders + IMU)')
    ax.scatter([r['g_x'][0]], [r['g_y'][0]], c='green', s=100, marker='o', zorder=5, label='Start')
    ax.scatter([r['g_x'][-1]], [r['g_y'][-1]], c='red', s=100, marker='x', zorder=5, label='End')
    ax.set_title('Path vs ground truth')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='center left', bbox_to_anchor=(1.01, 0.5))
    ax.grid(True)
    ax.set_aspect('equal', adjustable='datalim')

    fig.suptitle('Husky sensor validation (green = straight, blue = turn)', fontsize=14)
    fig.tight_layout()
    fig.savefig(path, dpi=150)

    # Summary as its own image
    lines = len(r['summary'].splitlines())
    sfig = plt.figure(figsize=(8, 0.24 * lines + 0.6))
    sfig.text(0.04, 0.98, r['summary'], va='top', ha='left', family='monospace', fontsize=11)
    sfig.savefig(summary_path, dpi=150, facecolor='white', bbox_inches='tight', pad_inches=0.3)

    if show:
        plt.show()
    plt.close(fig)
    plt.close(sfig)


# ============================================================ ROS node

class ValidationRun(Node):

    def __init__(self):
        super().__init__('validation_run', parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        cx, cy = p('obstacle_x', 4.0).value, p('obstacle_y', 0.0).value
        sx, sy = p('obstacle_size_x', 1.0).value, p('obstacle_size_y', 1.0).value
        self.cfg = {
            'linear_speed': p('linear_speed', 0.3).value,
            'straight_distance': p('straight_distance', 2.5).value,
            'angular_speed': p('angular_speed', 0.5).value,
            'turn_angle': math.radians(p('turn_angle_deg', 90.0).value),
            'second_straight': p('second_straight', 1.5).value,   # 0 to skip
            'settle_time': p('settle_time', 2.0).value,
            'stop_range': p('stop_range', 0.8).value,   # safety: forward lidar range
            'timeout': p('timeout', 120.0).value,       # sim seconds
            'wheel_radius': p('wheel_radius', 0.1651).value,
            'track': p('track', 0.5708).value,
            'box': (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2),
        }
        # Empty = husky_validation/ next to the workspace
        self.output_dir = os.path.expanduser(p('output_dir', '').value) or default_output_dir()
        self.show_plot = p('show_plot', True).value

        self.gt, self.odom, self.ekf, self.imu, self.enc = [], [], [], [], []
        self.enc_names = None
        self.scan_t, self.scan_ranges, self.scan_meta = [], [], None
        self.events = []
        self.wall = [0.0, 0.0]

        self.gt_latest = None
        self.gt_last_yaw = None
        self.gt_yaw_unwrapped = 0.0
        self.latest_scan = None
        self.phase = 'wait'
        self.phase_start = 0.0
        self.done = False
        self.wait_wall_start = time.time()
        self.warned_missing = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 50)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        # Optional: only present when the EKF is running (sim.launch.py ekf:=true)
        self.create_subscription(Odometry, 'odometry/filtered', self.on_ekf, 50)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 50)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.step)
        self.get_logger().info('Waiting for all sensor topics and /clock ...')

    # ---------------------------------------------------------- recording
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, header):
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        return t if t > 0.0 else self.now()

    def on_gt(self, msg):
        p, tw = msg.pose.pose, msg.twist.twist
        yaw = yaw_from_quat(p.orientation)
        if self.gt_last_yaw is None:
            self.gt_yaw_unwrapped = yaw
        else:
            self.gt_yaw_unwrapped += wrap(yaw - self.gt_last_yaw)
        self.gt_last_yaw = yaw
        self.gt_latest = (p.position.x, p.position.y)
        self.gt.append([self.stamp(msg.header), p.position.x, p.position.y, yaw,
                        tw.linear.x, tw.angular.z])

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom.append([self.stamp(msg.header), p.position.x, p.position.y,
                          yaw_from_quat(p.orientation)])

    def on_ekf(self, msg):
        p = msg.pose.pose
        self.ekf.append([self.stamp(msg.header), p.position.x, p.position.y,
                         yaw_from_quat(p.orientation)])

    def on_imu(self, msg):
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.imu.append([self.stamp(msg.header), yaw_from_quat(msg.orientation), w.z, a.x, a.y, a.z])

    def on_joints(self, msg):
        if self.enc_names is None:
            self.enc_names = sorted(n for n in msg.name if 'wheel' in n)
            self.get_logger().info(f'Encoders: {", ".join(self.enc_names)}')
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in self.enc_names):
            return
        pos = [msg.position[index[n]] for n in self.enc_names]
        vel = [msg.velocity[index[n]] if len(msg.velocity) > index[n] else 0.0 for n in self.enc_names]
        self.enc.append([self.stamp(msg.header), *pos, *vel])

    def on_scan(self, msg):
        if self.scan_meta is None:
            self.scan_meta = {'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                              'range_min': msg.range_min, 'range_max': msg.range_max}
        ranges = np.array(msg.ranges, dtype=float)
        self.latest_scan = ranges
        self.scan_t.append(self.stamp(msg.header))
        self.scan_ranges.append(ranges)

    def forward_range(self):
        if self.latest_scan is None or self.scan_meta is None:
            return math.inf
        i0 = int(round(-self.scan_meta['angle_min'] / self.scan_meta['angle_increment']))
        r = self.latest_scan[i0] if 0 <= i0 < len(self.latest_scan) else math.inf
        return r if math.isfinite(r) else math.inf

    # ---------------------------------------------------------- test sequence
    def set_phase(self, name):
        now = self.now()
        self.events.append((now, name))
        self.phase, self.phase_start = name, now
        if name == 'settle':
            self.wall[0] = time.time()
        if name == 'done':
            self.wall[1] = time.time()
            self.done = True
        self.get_logger().info(f'[{now:8.2f} s] phase -> {name}')

    def step(self):
        cfg, cmd = self.cfg, Twist()
        if self.phase == 'wait':
            streams = {'/ground_truth/odom': self.gt, '/odom': self.odom, '/imu': self.imu,
                       '/joint_states': self.enc, '/scan': self.scan_t}
            missing = [name for name, buf in streams.items() if not buf]
            if not missing:
                self.set_phase('settle')
            elif not self.warned_missing and time.time() - self.wait_wall_start > 15.0:
                self.get_logger().warn(f'Still no data on: {", ".join(missing)}. Is the simulation running?')
                self.warned_missing = True
            return

        now = self.now()
        elapsed = now - self.phase_start
        if self.phase != 'done' and now - self.events[0][0] > cfg['timeout']:
            self.get_logger().warn('Timeout reached, ending the test early')
            self.set_phase('done')

        if self.phase == 'settle':
            if elapsed >= cfg['settle_time']:
                self.start_xy = self.gt_latest
                self.set_phase('straight')
        elif self.phase in ('straight', 'straight2'):
            target = cfg['straight_distance'] if self.phase == 'straight' else cfg['second_straight']
            stop = 'stop1' if self.phase == 'straight' else 'stop3'
            dist = math.hypot(self.gt_latest[0] - self.start_xy[0], self.gt_latest[1] - self.start_xy[1])
            if dist >= target:
                self.set_phase(stop)
            elif self.forward_range() < cfg['stop_range']:
                self.get_logger().warn(f'Obstacle closer than {cfg["stop_range"]} m, stopping early')
                self.set_phase(stop)
            else:
                cmd.linear.x = cfg['linear_speed']
        elif self.phase == 'stop1':
            if elapsed >= cfg['settle_time']:
                self.turn_yaw0 = self.gt_yaw_unwrapped
                self.set_phase('turn')
        elif self.phase == 'turn':
            if abs(self.gt_yaw_unwrapped - self.turn_yaw0) >= cfg['turn_angle']:
                self.set_phase('stop2')
            else:
                cmd.angular.z = cfg['angular_speed']
        elif self.phase == 'stop2':
            if elapsed >= cfg['settle_time']:
                if cfg['second_straight'] > 0.0:
                    self.start_xy = self.gt_latest
                    self.set_phase('straight2')
                else:
                    self.set_phase('done')
        elif self.phase == 'stop3':
            if elapsed >= cfg['settle_time']:
                self.set_phase('done')
        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------- results
    def stop_robot(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())

    def abort(self):
        if self.events and not self.done:
            self.set_phase('done')

    def recorded(self):
        return {'gt': self.gt, 'odom': self.odom, 'ekf': self.ekf, 'imu': self.imu, 'enc': self.enc,
                'enc_names': self.enc_names, 'scan_t': self.scan_t, 'scan_ranges': self.scan_ranges,
                'scan_meta': self.scan_meta, 'events': self.events, 'wall': self.wall}

    def finish(self):
        data = self.recorded()
        result = analyze(data, self.cfg)
        out_dir = os.path.join(self.output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)
        save_csv(data, result, out_dir)
        figure = os.path.join(out_dir, 'sensor_validation_results.png')
        summary_image = os.path.join(out_dir, 'validation_summary.png')
        if not self.ekf:
            self.get_logger().warn('No /odometry/filtered received: EKF lines are left out. '
                                   'Launch the sim with ekf:=true (the default).')
        self.get_logger().info('\n' + result['summary'])
        self.get_logger().info(f'Saved figures, CSV files and summary to {out_dir}')
        show = self.show_plot and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        make_figure(result, figure, summary_image, show)


def main():
    rclpy.init()
    node = ValidationRun()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted: analysing what was recorded so far')
        node.abort()
    try:
        node.stop_robot()
    except Exception:  # context already shut down by Ctrl+C
        print('Could not send the stop command. Run: '
              'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"')
    try:
        if len(node.events) >= 2:
            node.finish()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
