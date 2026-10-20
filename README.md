# husky_sensor_sim

A Husky A200 in **Gazebo Fortress** on **ROS 2 Humble / Ubuntu 22.04**, with:

| Sensor | Where | ROS topic | Type |
|---|---|---|---|
| Wheel encoders (4) | each wheel joint | `/joint_states` | `sensor_msgs/JointState` (angle rad, velocity rad/s) |
| Encoder odometry | from DiffDrive | `/odom` + `/tf` | `nav_msgs/Odometry` |
| 2D lidar (360 beams, 20 m) | top plate, exactly x=0 y=0 | `/scan` | `sensor_msgs/LaserScan` |
| IMU (100 Hz) | robot centre | `/imu` | `sensor_msgs/Imu` |
| Ground truth | simulator | `/ground_truth/odom` | `nav_msgs/Odometry` |

---

## 1. Install dependencies

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
                 ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
                 python3-numpy python3-matplotlib
```

## 2. Build

```bash
mkdir -p ~/husky_ws/src
cp -r husky_sensor_sim ~/husky_ws/src/
cd ~/husky_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Run

**Terminal 1: simulation, bridge, RViz**
```bash
ros2 launch husky_sensor_sim sim.launch.py       
```

**Terminal 2: sensor check**
```bash
ros2 run husky_sensor_sim sensor_check --ros-args -p use_sim_time:=true
```

**Terminal 3: drive**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or run a scripted straight drive of about 2 m.
```bash
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 4. What you should see

**Right after start: robot at rest at the origin:**

| Check | Expected |
|---|---|
| lidar forward beam | ≈ 3.50 m (±0.01 m noise) |
| lidar closest return | ≈ 3.50 m |
| beams on obstacle | ~16 |
| IMU accel | x ≈ 0, y ≈ 0, **z ≈ +9.81 m/s²** |
| IMU gyro | ≈ 0 rad/s |
| encoder angles | 0 rad, 0 ticks |

**After the 2 m straight drive:**
- encoder distance ≈ ground-truth distance (error of a few mm)
- forward lidar beam ≈ 3.5 − distance driven
- the four wheel angles ≈ distance / 0.1651 rad

**After turning in place:**
- IMU gyro-integrated heading tracks the true heading closely.
- **Encoder** heading overestimates the turn. This is skid-steer behaviour, and the physical Husky has the same problem. It's the classic reason to fuse IMU yaw with wheel odometry.

Quick checks without the node:
```bash
ros2 topic hz /scan            # ~10 Hz
ros2 topic hz /imu             # ~100 Hz
ros2 topic echo /joint_states --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once --field ranges
```

## 5. Automated validation run

`validation_run` drives a fixed test and compares every sensor against ground truth:
settle 2 s → straight 2.5 m → stop → turn 90° in place → stop → straight 1.5 m → stop.

Start the simulation fresh (robot at the origin), then in a second terminal:
```bash
ros2 run husky_sensor_sim validation_run
```

Results go to `husky_validation/<date_time>/` in the folder that contains your workspace (for `~/projects/husky_ws` that is `~/projects/husky_validation/`):
- `sensor_validation_results.png`: heading, lidar range, wheel velocities, robot velocity, path, summary
- `summary.txt`: all metrics
- CSV files for ground truth, odometry, IMU, encoders, lidar and phases (sim time, t = 0 at test start)

Parameters (`--ros-args -p name:=value`):

| Parameter | Default |
|---|---|
| `linear_speed` / `straight_distance` | 0.3 m/s / 2.5 m |
| `angular_speed` / `turn_angle_deg` | 0.5 rad/s / 90 |
| `second_straight` | 1.5 m (0 skips it) |
| `settle_time` | 2.0 s |
| `stop_range` | 0.8 m (safety stop on the forward lidar beam) |
| `output_dir` | `husky_validation/` next to the workspace |
| `show_plot` | true |

## Files

```
husky_sensor_sim/
├── launch/sim.launch.py          Gazebo + spawn + robot_state_publisher + bridge + RViz
├── urdf/husky.urdf.xacro         robot, encoders (JointStatePublisher), lidar, IMU, DiffDrive, ground truth
├── worlds/husky_world.sdf        ground plane + 1 m cube
├── rviz/husky.rviz               robot, TF, scan, odometry
├── husky_sensor_sim/sensor_check.py     live report: every sensor vs ground truth
└── husky_sensor_sim/validation_run.py   automated test: records, analyses, plots, saves CSV
```
