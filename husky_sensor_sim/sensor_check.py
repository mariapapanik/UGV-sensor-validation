"""Check the simulated wheel encoders, lidar and IMU against Gazebo ground truth.

Ground truth comes from Gazebo's OdometryPublisher system (/ground_truth/odom),
which reports the robot's true world pose. Counters start from zero when this
node starts; restart it to reset them.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ray_box_distance(ox, oy, dx, dy, xmin, xmax, ymin, ymax):
    """Distance along a 2D ray from (ox, oy) in direction (dx, dy) to an axis-aligned box, or inf."""
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_near, t_far = max(t_near, t1), min(t_far, t2)
    if t_far < 0.0 or t_near > t_far:
        return math.inf
    return t_near if t_near >= 0.0 else t_far


def point_box_distance(px, py, xmin, xmax, ymin, ymax):
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.1651).value
        self.track = self.declare_parameter('track', 0.5708).value
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 1024).value
        # Must match the obstacle_box model in worlds/husky_world.sdf
        cx = self.declare_parameter('obstacle_x', 4.0).value
        cy = self.declare_parameter('obstacle_y', 0.0).value
        sx = self.declare_parameter('obstacle_size_x', 1.0).value
        sy = self.declare_parameter('obstacle_size_y', 1.0).value
        self.box = (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
        period = self.declare_parameter('report_period', 1.0).value

        self.joint_start = None   # name -> position when the node started
        self.joints = {}          # name -> (position, velocity)

        self.gt_pose = None       # (x, y, yaw) in world
        self.gt_twist = None      # (forward speed, yaw rate)
        self.gt_dist = 0.0        # signed distance driven along the heading
        self.gt_dyaw = 0.0        # unwrapped heading change

        self.odom_pose = None     # DiffDrive odometry (computed from the encoders)

        self.imu = None
        self.imu_last_stamp = None
        self.imu_dyaw = 0.0       # integrated gyro z

        self.scan = None

        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_ground_truth, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(period, self.report)

    # ------------------------------------------------------------------ callbacks
    def on_joints(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.joints[name] = (pos, vel)
        if self.joint_start is None:
            self.joint_start = {name: pos for name, (pos, _) in self.joints.items()}

    def on_ground_truth(self, msg):
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        if self.gt_pose is not None:
            px, py, pyaw = self.gt_pose
            self.gt_dist += (x - px) * math.cos(pyaw) + (y - py) * math.sin(pyaw)
            self.gt_dyaw += wrap(yaw - pyaw)
        self.gt_pose = (x, y, yaw)
        self.gt_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_imu(self, msg):
        t = stamp_sec(msg.header.stamp)
        if self.imu_last_stamp is not None and t > self.imu_last_stamp:
            self.imu_dyaw += msg.angular_velocity.z * (t - self.imu_last_stamp)
        self.imu_last_stamp = t
        self.imu = msg

    def on_scan(self, msg):
        self.scan = msg

    # ------------------------------------------------------------------ report
    def report(self):
        lines = ['', '=' * 78]
        if self.gt_pose is None:
            lines.append('Waiting for /ground_truth/odom ...')
        else:
            x, y, yaw = self.gt_pose
            lines.append(f'GROUND TRUTH  x={x:7.3f} m  y={y:7.3f} m  yaw={math.degrees(yaw):7.2f} deg')
        lines += self.encoder_report()
        lines += self.imu_report()
        lines += self.lidar_report()
        self.get_logger().info('\n'.join(lines))

    def encoder_report(self):
        if not self.joint_start:
            return ['ENCODERS  waiting for /joint_states ...']
        lines = ['ENCODERS  (wheel angle -> ticks -> arc length)']
        left, right = [], []
        for name in sorted(self.joints):
            pos, vel = self.joints[name]
            angle = pos - self.joint_start.get(name, pos)
            ticks = int(angle / (2.0 * math.pi) * self.ticks_per_rev)
            arc = angle * self.wheel_radius
            (left if 'left' in name else right).append(arc)
            lines.append(f'  {name:<24}{angle:9.3f} rad {ticks:9d} ticks {arc:8.3f} m'
                         f'  {vel * self.wheel_radius:6.2f} m/s')
        if left and right and self.gt_pose is not None:
            l_arc, r_arc = sum(left) / len(left), sum(right) / len(right)
            enc_dist = (l_arc + r_arc) / 2.0
            enc_dyaw = (r_arc - l_arc) / self.track
            lines.append(f'  distance   encoders {enc_dist:8.3f} m    truth {self.gt_dist:8.3f} m'
                         f'    error {enc_dist - self.gt_dist:+.3f} m')
            lines.append(f'  heading    encoders {math.degrees(enc_dyaw):8.2f} deg  '
                         f'truth {math.degrees(self.gt_dyaw):8.2f} deg  '
                         f'error {math.degrees(enc_dyaw - self.gt_dyaw):+.2f} deg  (skid-steer slip)')
        if self.odom_pose is not None and self.gt_pose is not None:
            ox, oy, oyaw = self.odom_pose
            gx, gy, gyaw = self.gt_pose
            lines.append(f'  /odom pose error  dx={ox - gx:+.3f} m  dy={oy - gy:+.3f} m'
                         f'  dyaw={math.degrees(wrap(oyaw - gyaw)):+.2f} deg')
        return lines

    def imu_report(self):
        if self.imu is None:
            return ['IMU       waiting for /imu ...']
        imu = self.imu
        a, w = imu.linear_acceleration, imu.angular_velocity
        lines = ['IMU',
                 f'  accel   x={a.x:7.3f}  y={a.y:7.3f}  z={a.z:7.3f} m/s^2'
                 f'   |a|={math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z):6.3f}  (rest: z ~ +9.81)',
                 f'  gyro    x={w.x:7.3f}  y={w.y:7.3f}  z={w.z:7.3f} rad/s']
        if self.gt_twist is not None:
            lines.append(f'  yaw rate   imu {w.z:8.3f} rad/s  truth {self.gt_twist[1]:8.3f} rad/s'
                         f'  error {w.z - self.gt_twist[1]:+.3f}')
            lines.append(f'  heading    gyro-integrated {math.degrees(self.imu_dyaw):8.2f} deg'
                         f'  truth {math.degrees(self.gt_dyaw):8.2f} deg'
                         f'  error {math.degrees(self.imu_dyaw - self.gt_dyaw):+.2f} deg')
        if self.gt_pose is not None:
            imu_yaw = yaw_from_quat(imu.orientation)
            lines.append(f'  orientation yaw  imu {math.degrees(imu_yaw):8.2f} deg'
                         f'  truth {math.degrees(self.gt_pose[2]):8.2f} deg')
        return lines

    def lidar_report(self):
        if self.scan is None or self.gt_pose is None:
            return ['LIDAR     waiting for /scan and ground truth ...']
        scan = self.scan
        # The lidar sits at x=0, y=0 of base_footprint, so its position is the robot position
        x, y, yaw = self.gt_pose
        errors, missed, unexpected = [], 0, 0
        for i, r in enumerate(scan.ranges):
            beam = yaw + scan.angle_min + i * scan.angle_increment
            expected = ray_box_distance(x, y, math.cos(beam), math.sin(beam), *self.box)
            expects_hit = scan.range_min <= expected <= scan.range_max
            got_hit = math.isfinite(r) and scan.range_min <= r <= scan.range_max
            if expects_hit and got_hit:
                errors.append(r - expected)
            elif expects_hit:
                missed += 1
            elif got_hit:
                unexpected += 1

        lines = [f'LIDAR     frame={scan.header.frame_id}  beams={len(scan.ranges)}'
                 f'  incr={math.degrees(scan.angle_increment):.2f} deg']

        i0 = round(-scan.angle_min / scan.angle_increment)
        if 0 <= i0 < len(scan.ranges):
            r0 = scan.ranges[i0]
            e0 = ray_box_distance(x, y, math.cos(yaw), math.sin(yaw), *self.box)
            err = f'{r0 - e0:+.3f} m' if math.isfinite(r0) and math.isfinite(e0) else 'n/a'
            lines.append(f'  forward beam   measured {r0:7.3f} m   expected {e0:7.3f} m   error {err}')

        finite = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        if finite:
            lines.append(f'  closest return measured {min(finite):7.3f} m   expected '
                         f'{point_box_distance(x, y, *self.box):7.3f} m (lidar -> nearest box point)')

        if errors:
            mean_abs = sum(abs(e) for e in errors) / len(errors)
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            worst = max(errors, key=abs)
            lines.append(f'  {len(errors)} beams on obstacle   mean|err| {mean_abs:.3f} m'
                         f'   rms {rms:.3f} m   worst {worst:+.3f} m')
        lines.append(f'  beams expected to hit but missed: {missed}   unexpected hits: {unexpected}')
        return lines


def main():
    rclpy.init()
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
