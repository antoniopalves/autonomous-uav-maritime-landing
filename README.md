\# Autonomous UAV Landing on a Moving Maritime Vessel



Autonomous landing of a PX4-controlled UAV on a WAM-V marker target in a VRX (Virtual RobotX) simulation, under configurable sea states and vessel motion.



<p align="center">

&#x20; <img src="assets/drone.png" width="700" alt="UAV above the WAM-V landing target in the VRX simulation">

</p>



\## Demo



\### Stationary vessel



<!-- Paste the GitHub video link for the stationary landing here (on its own line) -->



\### Moving vessel



<!-- Paste the GitHub video link for the moving landing here (on its own line) -->



\## System Requirements



\- \*\*OS:\*\* Ubuntu 22.04 LTS (native or WSL2)

\- \*\*Middleware:\*\* ROS 2 Humble Hawksbill

\- \*\*Autopilot:\*\* PX4 Autopilot SITL (v1.14) + MAVROS 2

\- \*\*Simulator:\*\* Gazebo Sim / Virtual RobotX (VRX 2023)

\- \*\*Python dependencies:\*\* `rclpy`, `numpy`, `opencv-python` (`cv2.aruco`), `cv\_bridge`



\## Setup \& File Integration



To integrate these files into an existing VRX and PX4 SITL workspace:



\### 1. World Configuration (`sydney\_regatta.sdf`)



Copy `simulation/worlds/sydney\_regatta.sdf` into your VRX worlds directory (`vrx\_gz/worlds/`).



<p align="center">

&#x20; <img src="assets/world.png" width="500" alt="Sydney Regatta world in Gazebo">

</p>



To configure different sea states, edit the `vrx::PublisherPlugin` block (`/vrx/wavefield/parameters`) at the bottom of `sydney\_regatta.sdf`:



| Sea state | Description | `gain` | `period` | `steepness` |

|-----------|-------------|--------|----------|-------------|

| SS 3 | Slight   | 0.25 | 5.0 | 0.0 |

| SS 4 | Moderate | 0.75 | 7.0 | 0.2 |

| SS 5 | Rough    | 1.25 | 9.0 | 0.4 |



\### 2. WAM-V Target Configuration (`wamv\_gazebo.urdf.xacro`)



Replace the default `wamv\_gazebo.urdf.xacro` in `wamv\_gazebo/urdf/` with `simulation/models/wamv/wamv\_gazebo.urdf.xacro`.



> \*\*Note:\*\* Update the `<albedo\_map>` URI path on line 196 to match the local path of `aruco\_marker.png` on your machine.



<p align="center">

&#x20; <img src="aruco\_marker.png" width="200" alt="ArUco marker used as the landing target on the WAM-V">

&#x20; <br>

&#x20; <em>ArUco marker (<code>aruco\_marker.png</code>) used as the landing target on the WAM-V.</em>

</p>



\### 3. UAV \& Sensor Models (`mono\_cam` \& `x500\_mono\_cam\_down`)



Copy `simulation/models/mono\_cam/model.sdf` and `simulation/models/x500\_mono\_cam\_down/model.sdf` into your PX4 Gazebo models directory (`PX4-Autopilot/Tools/simulation/gz/models/`).



\## Running the Autonomous Landing Node



1\. Launch the VRX simulation world, PX4 SITL instance, `ros\_gz\_bridge`, and MAVROS.

2\. In `src/land\_debug.py`, select the desired vessel motion mode on line 50:

&#x20;  - `self.boat\_mode = 1`: Stationary vessel (0 N thrust)

&#x20;  - `self.boat\_mode = 2`: Moving vessel (90 N continuous forward thrust per aft thruster)

3\. Execute the autonomous landing node:



```bash

python3 src/land\_debug.py

```



Upon completion (Phase 5, Motor Disarm), the node automatically computes and prints the terminal landing report and CSV summary line to the terminal.



\## Author



\*\*António Maria de Oliveira Porto e Alcoforado Alves\*\*



Master in Electrical and Computer Engineering, NOVA School of Science and Technology (NOVA FCT), 2026.

