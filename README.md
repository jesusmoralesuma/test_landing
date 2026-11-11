# Vision-based autonomous following and landing of a UAV on a moving ground platform

ROS code that lets a multirotor UAV detect, follow and land on a moving unmanned ground vehicle (UGV) using ArUco fiducial markers and a single on-board camera. It accompanies the article:

> J. Morales, I. Castelo, R. Serra, P. U. Lima and M. Basiri, "Vision-Based Autonomous Following of a Moving Platform and Landing for an Unmanned Aerial Vehicle", *Sensors*, vol. 23, no. 2, 829, 2023. https://doi.org/10.3390/s23020829

The code was written in June and July 2022 at the Institute for Systems and Robotics (ISR/LARSyS) of Instituto Superior Técnico, Lisbon, within the EU project DURABLE. It is research code released as is, to document the method and make the experiments reproducible. It is not actively maintained.

## How it works

1. The `aruco_detect` node estimates the pose of the markers on the landing platform from the camera images and publishes it on `/fiducial_transforms`. Four markers are used (IDs 946, 227, 168 and 55); their offsets on the platform are set as static transforms in the launch files.
2. `follow_UGV_trajectory_trap_unified.py` (node `drone_controller`) turns those detections into an estimate of the UAV pose relative to the platform: position and yaw.
3. A trapezoidal velocity profile (`trajectory.py`) plans the motion from the current relative pose to the requested set-point.
4. Four decoupled controllers (x, y, z and yaw) add a feedback correction to the planned velocity, and the result is sent to the autopilot as a velocity set-point through MAVROS. The autopilot runs the inner control loop in `GUIDED` mode.

### Topics

| Direction | Topic | Type | Purpose |
| :--- | :--- | :--- | :--- |
| Subscribed | `/fiducial_transforms` | `fiducial_msgs/FiducialTransformArray` | Marker detections |
| Subscribed | `/mavros/state` | `mavros_msgs/State` | Autopilot flight mode |
| Subscribed | `/uav_setpoint` | `geometry_msgs/Twist` | Desired pose relative to the marker frame (x, y, z, yaw) |
| Subscribed | `/uav_mode` | `std_msgs/Int8` | 0: teleoperated, 1: autonomous |
| Published | `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | Velocity command for the autopilot |
| Published | `/estimation/xyzy` | `geometry_msgs/Twist` | Estimated relative pose |
| Published | `/observations/xyz`, `/observations/rpy` | `std_msgs/Float64MultiArray` | Raw marker observations |

## Repository contents

| Path | Description |
| :--- | :--- |
| `follow_UGV_trajectory_trap_unified.py` | Controller node: pose estimation from the markers, trajectory planning and velocity commands |
| `trajectory.py` | Trapezoidal velocity profile generator for x, y, z and yaw |
| `test_trajectory.py` | Offline plots of the generated profiles |
| `data_read_unified.py` | Reads the recorded rosbag files and plots the experiments |
| `simulation_unified.launch` | Launches the simulation: DURABLE Gazebo world, MAVROS, marker detection, static transforms and teleoperation |
| `transforms.launch` | Launches the real UAV: MAVROS, RealSense camera, marker detection and static transforms |
| `models/iris_camera/` | Gazebo model of the Iris multirotor with a camera (third-party model, see below) |
| `info_simulation.txt`, `info_real.txt` | Original step-by-step notes for running the simulation and the real experiments |

## Requirements

- ROS 1 with Python (`rospy`, `tf`, `tf2_ros`, `rosbag`), plus `numpy` and `matplotlib`. The code was used in 2022 with ROS Melodic or Noetic (the exact distribution was not recorded) and contains nothing specific to either. The DURABLE simulator was originally integrated with ROS Kinetic.
- On-board computer of the real UAV: NVIDIA Jetson Xavier NX.
- ROS packages: `mavros`, `aruco_detect` and `fiducial_msgs` (from the `fiducials` stack), `hector_imu_attitude_to_tf`, and a gamepad teleoperation package (`hexa_teleop` in the launch files).
- Simulation only: Gazebo, ArduPilot software in the loop (ArduCopter) and the DURABLE multi-robot simulator, which provides the `durable_gazebo_simulation` package: https://github.com/durable-ist/Multi_Robot_Simulation
- Real UAV only: an ArduPilot autopilot connected to the on-board computer (the launch file expects `/dev/ttyTHS0` at 921600 baud) and an Intel RealSense camera with `realsense2_camera`.

## Running the simulation

The commands below assume this repository is the working directory.

1. Launch the simulator, MAVROS, the marker detector and the teleoperation node:
   ```bash
   roslaunch simulation_unified.launch
   ```
2. Start ArduCopter software in the loop from the ArduPilot source tree:
   ```bash
   cd <ardupilot>/ArduCopter
   ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console -I0 -L Evora --mavproxy-args="--streamrate=-1"
   ```
3. Publish the autopilot data at 15 Hz:
   ```bash
   rosrun mavros mavsys rate --all 15
   ```
4. When the console reports `EKF3 IMU1 is using GPS`, take off from the ArduCopter console:
   ```
   GUIDED
   arm throttle
   takeoff 3
   ```
5. Run the controller:
   ```bash
   python follow_UGV_trajectory_trap_unified.py
   ```
6. Switch between teleoperated and autonomous mode with the gamepad (L1: teleoperation, R1: autonomous).
7. Move the ground robot by publishing velocity commands on `/jackal2/jackal_velocity_controller/cmd_vel`, and change the desired relative pose of the UAV by publishing on `/uav_setpoint`. For example, to hover 2 m above the platform:
   ```bash
   rostopic pub /uav_setpoint geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 2.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

## Running on the real UAV

On the on-board computer, over SSH:

1. Give access to the autopilot serial port and launch the drivers:
   ```bash
   sudo chmod 666 /dev/ttyTHS0
   roslaunch transforms.launch
   ```
2. Optionally record the experiment:
   ```bash
   rosbag record /tf /tf_static /fiducial_images/compressed /fiducial_transforms /mavros/imu/data /mavros/setpoint_velocity/cmd_vel_unstamped /mavros/state /estimation/xyzy /observations/rpy /observations/xyz /uav_setpoint /uav_mode
   ```
3. Run the controller:
   ```bash
   python follow_UGV_trajectory_trap_unified.py
   ```

On the ground station, launch the gamepad teleoperation node. A safety pilot must be able to take manual control at any time.

## Analysing the experiments

`data_read_unified.py` reads a recorded rosbag and plots the estimated pose, the set-points and the velocity commands. Set `bagdir` and `bagfile` at the top of the script to point to your recording. The rosbag files of the published experiments are not included in this repository.

`test_trajectory.py` plots the trapezoidal profiles offline. It imports a helper function from `data_read_unified.py`, which reads the configured rosbag when imported, so a recording must be available to run it.

## Citation

If you use this code, please cite the article:

```bibtex
@article{morales2023vision,
  author  = {Morales, Jes{\'u}s and Castelo, Isabel and Serra, Rodrigo and Lima, Pedro U. and Basiri, Meysam},
  title   = {Vision-Based Autonomous Following of a Moving Platform and Landing for an Unmanned Aerial Vehicle},
  journal = {Sensors},
  volume  = {23},
  number  = {2},
  pages   = {829},
  year    = {2023},
  doi     = {10.3390/s23020829}
}
```

## Funding

This work was funded by the ISR/LARSyS Strategic Funding through the FCT project UIDB/50009/2020, the DURABLE project under the Interreg Atlantic Area Programme through the European Regional Development Fund (ERDF), the Andalusian project UMA18-FEDERJA-090 and the University of Málaga Research Plan.

## Third-party content

`models/iris_camera/` is a Gazebo model derived from the `iris_with_ardupilot` model of the ArduPilot Gazebo plugin (https://github.com/khancyr/ardupilot_gazebo), which is distributed under the GNU General Public License v3.0. Its original authors and maintainer are listed in `models/iris_camera/model.config`. This directory is not covered by the BSD licence of the rest of the repository: it is redistributed under the GPL-3.0 licence of the original model.

## Licence

The code in this repository is released under the BSD 3-Clause License. See [LICENSE](LICENSE). The only exception is `models/iris_camera/`, as explained above.

## Contact

Authors: Jesús Morales (Universidad de Málaga), Isabel Castelo, Rodrigo Serra, Pedro U. Lima and Meysam Basiri (Institute for Systems and Robotics, Instituto Superior Técnico, Universidade de Lisboa).

Contact: Jesús Morales, Departamento de Ingeniería de Sistemas y Automática, Universidad de Málaga. ORCID: [0000-0003-1095-4775](https://orcid.org/0000-0003-1095-4775)
