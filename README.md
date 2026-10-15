# Autonomous UAV Landing on a Moving Maritime Vessel

Autonomous landing of a PX4-controlled UAV on a WAM-V marker target in a VRX (Virtual RobotX) simulation, under configurable sea states and vessel motion.

## System Requirements

- **OS:** Ubuntu 22.04 LTS (native or WSL2)
- **Middleware:** ROS 2 Humble Hawksbill
- **Autopilot:** PX4 Autopilot SITL (v1.14) + MAVROS 2
- **Simulator:** Gazebo Sim / Virtual RobotX (VRX 2023)
- **Python dependencies:** `rclpy`, `numpy`, `opencv-python` (`cv2.aruco`), `cv_bridge`

## Setup & File Integration

To integrate these files into an existing VRX and PX4 SITL workspace:

### 1. World Configuration (`sydney_regatta.sdf`)

Copy `simulation/worlds/sydney_regatta.sdf` into your VRX worlds directory (`vrx_gz/worlds/`).

To configure different sea states, edit the `vrx::PublisherPlugin` block (`/vrx/wavefield/parameters`) at the bottom of `sydney_regatta.sdf`:

| Sea state | Description | `gain` | `period` | `steepness` |
|-----------|-------------|--------|----------|-------------|
| SS 3 | Slight   | 0.25 | 5.0 | 0.0 |
| SS 4 | Moderate | 0.75 | 7.0 | 0.2 |
| SS 5 | Rough    | 1.25 | 9.0 | 0.4 |

### 2. WAM-V Target Configuration (`wamv_gazebo.urdf.xacro`)

Replace the default `wamv_gazebo.urdf.xacro` in `wamv_gazebo/urdf/` with `simulation/models/wamv/wamv_gazebo.urdf.xacro`.

> **Note:** Update the `<albedo_map>` URI path on line 196 to match the local path of `aruco_marker.png` on your machine.

<p align="center">
  <img src="aruco_marker.png" width="200" alt="ArUco marker used as the landing target on the WAM-V">
  <br>
  <em>ArUco marker (<code>aruco_marker.png</code>) used as the landing target on the WAM-V.</em>
</p>

### 3. UAV & Sensor Models (`mono_cam` & `x500_mono_cam_down`)

Copy `simulation/models/mono_cam/model.sdf` and `simulation/models/x500_mono_cam_down/model.sdf` into your PX4 Gazebo models directory (`PX4-Autopilot/Tools/simulation/gz/models/`).

## Running the Autonomous Landing Node

1. Launch the VRX simulation world, PX4 SITL instance, `ros_gz_bridge`, and MAVROS.
2. In `src/land_debug.py`, select the desired vessel motion mode on line 50:
   - `self.boat_mode = 1`: Stationary vessel (0 N thrust)
   - `self.boat_mode = 2`: Moving vessel (90 N continuous forward thrust per aft thruster)
3. Execute the autonomous landing node:

```bash
python3 src/land_debug.py
```

Upon completion (Phase 5, Motor Disarm), the node automatically computes and prints the terminal landing report and CSV summary line to the terminal.

## Author

**António Maria de Oliveira Porto e Alcoforado Alves**

Master in Electrical and Computer Engineering, NOVA School of Science and Technology (NOVA FCT), 2026.
