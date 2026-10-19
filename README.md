# husky_sensor_sim

A Husky A200 in **Gazebo Fortress** on **ROS 2 Humble / Ubuntu 22.04**, with:

| Sensor | Where | ROS topic | Type |
|---|---|---|---|
| Wheel encoders (4) | each wheel joint | `/joint_states` | `sensor_msgs/JointState` (angle rad, velocity rad/s) |
| Encoder odometry | from DiffDrive | `/odom` + `/tf` | `nav_msgs/Odometry` |
| 2D lidar (360 beams, 20 m) | top plate, exactly x=0 y=0 | `/scan` | `sensor_msgs/LaserScan` |
| IMU (100 Hz) | robot centre | `/imu` | `sensor_msgs/Imu` |
| Ground truth | simulator | `/ground_truth/odom` | `nav_msgs/Odometry` |

The world is flat ground with one static **1 m cube centred at (4, 0)**. The robot spawns at the origin facing +x, so the cube's near face is **3.5 m** in front of the lidar.

Robot kinematics match Clearpath's `husky_description` (wheel radius 0.1651 m, track 0.5708 m, wheelbase 0.512 m). The body is built from primitive shapes, so the package needs no meshes.

---

## 1. Install dependencies

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
                 ros-humble-rviz2 ros-humble-teleop-twist-keyboard
```

On Humble, `ros-humble-ros-gz` is built against Gazebo Fortress. Check that it is there:

```bash
ign gazebo --version
```

If `ign` is not found, install Fortress from the OSRF repo:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install ignition-fortress
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
ros2 launch husky_sensor_sim sim.launch.py          # rviz:=false to skip RViz
```

**Terminal 2: sensor check (prints a report every second)**
```bash
ros2 run husky_sensor_sim sensor_check --ros-args -p use_sim_time:=true
```

**Terminal 3: drive**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or run a scripted straight drive of about 2 m. Gazebo's DiffDrive **keeps the last command**, so always send the stop:
```bash
ros2 topic pub -r 10 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 4. What you should see

**Right after start (robot at rest at the origin):**

| Check | Expected |
|---|---|
| lidar forward beam | ≈ 3.50 m (±0.01 m noise) |
| lidar closest return | ≈ 3.50 m |
| beams on obstacle | ~16 (the cube spans about ±8°) |
| IMU accel | x ≈ 0, y ≈ 0, **z ≈ +9.81 m/s²** (an IMU measures gravity as upward specific force) |
| IMU gyro | ≈ 0 rad/s |
| encoder angles | 0 rad, 0 ticks |

**After the 2 m straight drive:**
- encoder distance ≈ ground-truth distance (error of a few mm)
- forward lidar beam ≈ 3.5 − distance driven
- the four wheel angles ≈ distance / 0.1651 rad

**After turning in place:**
- IMU gyro-integrated heading tracks the true heading closely.
- **Encoder** heading overestimates the turn. This is real skid-steer behaviour (the wheels slip sideways), and the physical Husky has the same problem. It's the classic reason to fuse IMU yaw with wheel odometry.

Quick checks without the node:
```bash
ros2 topic hz /scan            # ~10 Hz
ros2 topic hz /imu             # ~100 Hz
ros2 topic echo /joint_states --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once --field ranges
```

In RViz (fixed frame `odom`) the red scan points should form the cube's near face 3.5 m ahead.
In Gazebo you can show the lidar rays via the ⋮ menu → **Visualize Lidar** (topic `/scan`).

## 5. Tuning

| What | Where |
|---|---|
| Lidar beams / range / noise | `urdf/husky.urdf.xacro` → `<sensor name="lidar">` |
| IMU rate / noise | `urdf/husky.urdf.xacro` → `<sensor name="imu">` (set stddev to 0 for exact values) |
| Encoder ticks per revolution | `sensor_check` parameter `ticks_per_rev` (default 1024) |
| Obstacle position / size | `worlds/husky_obstacle.sdf`, **and** the `obstacle_*` params of `sensor_check` |

Example: if you move the box to (3, 2), run
`ros2 run husky_sensor_sim sensor_check --ros-args -p use_sim_time:=true -p obstacle_x:=3.0 -p obstacle_y:=2.0`.

Restart `sensor_check` to zero its counters.

## 6. Troubleshooting

- **No `/scan` messages, or Gazebo crashes when it starts in a VM or WSL2.** The GPU lidar needs OpenGL 3.3 (ogre2). Try `export LIBGL_ALWAYS_SOFTWARE=1` before launching. On WSL2, update your GPU driver so WSLg has hardware OpenGL.
- **The scan's `frame_id` is `husky/base_footprint/lidar` instead of `lidar_link`, and RViz shows nothing.** The `<ignition_frame_id>` tag wasn't honoured. Publish a static TF as a workaround:
  `ros2 run tf2_ros static_transform_publisher --frame-id lidar_link --child-frame-id husky/base_footprint/lidar`
  Do the same for the IMU if needed.
- **The robot doesn't spawn.** Make sure the world name in the `.sdf` (`husky_world`) matches `WORLD_NAME` in the launch file.
- **The robot doesn't move.** Check `ign topic -l | grep cmd_vel` and `ros2 topic info /cmd_vel`. The bridge must show a subscriber on the ROS side.

## Files

```
husky_sensor_sim/
├── launch/sim.launch.py          Gazebo + spawn + robot_state_publisher + bridge + RViz
├── urdf/husky.urdf.xacro         robot, encoders (JointStatePublisher), lidar, IMU, DiffDrive, ground truth
├── worlds/husky_obstacle.sdf     ground plane + 1 m cube at (4, 0)
├── rviz/husky.rviz               robot, TF, scan, odometry
└── husky_sensor_sim/sensor_check.py   compares every sensor against ground truth
```
