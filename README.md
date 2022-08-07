# TUM I2ROS Autonomous Drone
This is the repo for group project autonomous drone of team 3 in course introduction to ROS in summer semester 2022.

## Introduction
This project aims to perform cooperative UAV exploration in unity simulation environment with two drones. The range of the mapping area is 100m x 100m, the target resolution of the map is 1m.

Watch the video below to get an intuition of how this project works.

[![](https://img.youtube.com/vi/m8VRTH65I68/0.jpg)](https://www.youtube.com/watch?v=m8VRTH65I68)


## Quick Start
1. install dependencies
   ```bash
   sudo apt-get install ros-noetic-octomap* ros-noetic-navigation ros-noetic-gmapping
   ```
2. build the workspace
   ```bash
   cd drone_ws
   catkin build
   ```
   Note: if you encounter error during building, source the workspace and try again.
3. launch all in one
   ```bash
   roslaunch state_machine_pkg all_in_one.launch
   ```
   After launch, press space in the command line. The drones will take off and start exploring, then land automatically after the exploration is done.
   
   Note: You can launch each segment of our project separately, see segment description for detail.

## General structure
![plot](./images/structure.png)

The project basically contains 6 modules. 
- The **simulation** module read Sensor data from unity and send commands to it. 
- **Mapping** reads camera frame and depth image and publishes a global octomap and a projected 2d map. 
- **Planning** takes the map as input and listen to commands from exploration to perform path planning. 
- The **state machine** monitors drone state and user input and triggers control and exploration according to the current state. 
- **Control** read the drone state from simulation and send command back to make sure that the drone follows the path or commands from state machine. 
- **Exploration** takes the current generated map and drone state to decide where are next goals for drones to explore.

The following is a concrete node graph of the project.
![plot](./images/rosgraph.png)

## Remaining problems

- Sometimes the drone will still hit the wall because of not quick enough octomap generation
- The landing performance differs in different computer because we use a optional z-offset to control the landing smoothly. Due to performance variaty of different devices the optimal z-offset on different computer can be different.

---
---

The following is the introduction of each package in the project


# Simulation

This module communicate with unity environment. It reads sensor data from quadrotor and generate tf according to it. It also subscribes to command topics and send the commands to unity.
## Simulation package

### Run Unity and ROS on one machine:
```bash
roslaunch simulation simulation.launch
roslaunch controller_pkg circle_demo.launch 
```

### Run Unity on host machine and ROS on virtual machine:
1. Establish TCP port forwarding for port 9997 and 9998 between host and virtual machine.
2. Run unity on host machine.
3. Edit the ip address in `on_host_simulation.launch`.
4. Run the following commands on virtual machine:
    ```bash
    roslaunch simulation on_host_simulation.launch
    roslaunch controller_pkg circle_demo.launch 
    ```

### Nodes:
- Quadrotor / Quadrotor2:
  - unity_ros (read data from unity)
  - w_to_unity (send command to unity)
  - state_estimate_corruptor_node (add noise to sensor data)
  - base_to_world (broadcast dynamic tf)
  - static_transform_publisher * 4 (broadcast static tf)
- time_server (manage simulation time)
### Tf tree:
- world:
  - Quadrotor / Quadrotor2
    - Sensors
      - DepthCamera
      - RGBCameraLeft
      - RGBCameraRight

### Topic tree:
- clock (simulation time)
- Quadrotor / Quadrotor2:
  - clock
  - Sensors:
    - IMU: (Precise state)
      - pose
      - twist
    - DepthCamera / RGBCameraLeft / RGBCameraRight:
      - image_raw
      - camera_info
  - Estimation: (State with noise)
    - pose
    - twist
    - current_state
  - Commands: (Control signal)
    - rotor_speed_cmds

The simulation package uses simulation time, which is counted from the unity startup.

Subscribe to topics under `Estimation` or `IMU` to get the current state. 

Subscribe to topics under `DepthCamera` / `RGBCameraLeft` / `RGBCameraRight` to get the camera info and image. 

Publish to `Commands/rotor_speed_cmds` to control the drones.

# Map Service

This module firstly projects the depth image onto the RGB camera frames. Then generate RGB point clouds using the projected depth image and the rgb image. The rgb point clouds from two drones are then used to generate an OctoMap, which is finally projected into a 2d map.
## map package:
### Dependencies
- octomap*
### Quick launch
quick launch simulation and map:
```bash
# unity on linux
roslaunch map simulation_map.launch
# unity on host machine
roslaunch map on_host_simulation_map.launch
```
### Save map
```bash
rosrun octomap_server octomap_saver -f drones_map.bt 
octovis drones_map.bt 
```

### Nodes:
- Quadrotor / Quadrotor2:
  - register_depth 
    
    (Project depth images into RGBCamera frames)
  - register_point_cloud 
  
    (Generate point clouds from RGB and projected depth images)
  - republish_point_cloud 
   
    (Republish point clouds into one global topic)
- octomap_server 
  
  (Generate OctoMap and 2d map from RGB point clouds)

### Topic tree:
- Quadrotor / Quadrotor2:
  - Registered:
    - DepthCamera:
      - image_raw
      - camera_info
      - point_cloud
- point_cloud
- octomap_full
- octomap_binary
- projected_map
- occupied_cells_vis_array 
- octomap_point_cloud_centers 

## octomap_mapping package:
Mapping tools to be used with the OctoMap library, implementing a 3D occupancy grid mapping.
- original author: Armin Hornung
- original source: [octomap_mapping](http://wiki.ros.org/octomap_mapping)

### Modification
- Change setting to support color octomap.

# Path Planning

This module takes the projected map and a goal point in world frame as inputs, then publishes local plan of each quadrotor (the cropped global plan in local window) for the controller. Parameter tuning is the most important part of this module.
## planning package
### Dependencies

- navigation
- gmapping

### Quick launch
quick launch simulation, map, and planning:
```bash
# unity on linux
roslaunch planning simulation_map_path.launch
# unity on host machine
roslaunch planning on_host_simulation_map_path.launch
```

### Nodes:
- Quadrotor / Quadrotor2:
  - move_base (Motion planning service)
  - world_center_pub (Publish fake odometry)
  
### Topic tree:
- Quadrotor / Quadrotor2:
  - move_base:
    - TrajectoryPlannerROS:
      - global_plan (the cropped global plan as local plan)
    - NavfnROS:
      - plan (the global plan)

### Actions:
- Quadrotor / Quadrotor2:
  - move_base

# Exploration

This module firstly generates a costmap with the projected map, then each drone applies greedy frontier search that is defined by explore_lite. During the goal assignment, the explore_lite package is modified to reject goal points that are aloready signed to other drones. Furthermore, some special limitations are set when searching for frontiers and goal points to improve efficiency. A service server and a service client are added in order to communicate with the state machine. Explore can be triggered by state machine. The state of state machines can also be switched to "landing" by the exploration module.
## explore package

Contains launch files to start exploration.
### Main Dependencies

- explore_lite
- costmap_2d
- service_pkg

### Quick launch

After launching simulation, map, and planning:
```bash
roslaunch explore costmap_explore.launch
```

### Nodes
- costmap_server 
 
  (generate a costmap from projected map, the inflaction helps to filter out narro frontiers.)
- explore 
 
  (The core of the exploration, performs frontier search, goal assignment, communication to state machine etc.)

### Topic tree:
- costmap_server:
  - costmap:
    - ...
    - costmap
    - costmap_updates
- explore:
  - frontiers

### Services:
- explore:
  - switch_explore (switch explore on and off)

## explore_lite package

Greedy frontier explore algorithm for multi robots.

- original author: Jiri Horner
- original source: [explore_lite](http://wiki.ros.org/explore_lite)

### Modification
- add multi robot support.
- add service client of state machine to trigger landing.
- add parameterized explore range limit.
- add option to set goal only on frontier.
- add option to choose whether to autostart the explore.
- add service to switch explore on and off.


# Control

This module consists of controller_node and trajectory publisher. The PID parameters in controller and other dynamic coefficient are tuned such that drone can fly smoothly. The trajectory publisher subscribes to the state topic and calculate the desired velocity and poses in different flying stages based on trajectory planner. The controller node take these requests and compute a control command which is send to unity. An offset parameter is added to make the landing look more nature.
## controller package
traj_publisher for generating the desired poses and velocities
- pure path following 
- rotate while following path
controller_node for torque controlling.
- original author: Prof. Dr. Markus Ryll
### Quick launch
After launching simulation, map, planning, and explore:
```bash
roslaunch controller_pkg fly_drones.launch
```

### Nodes:
- Quadrotor / Quadrotor2:
  - controller_node 
    
    (In charge of the controlling of rotor torque)
  - traj_publisher 
    
    (cooperate with state machine, control the poses and velocity of drones in different flying stages)

### Topic tree:
- Quadrotor / Quadrotor2:
  - Commands:
    - desired_state
    - rotor_speed_cmds

# State machine

Manage drones with 5 states:
1. Initial state
  
    The drone stays still.
2. Launch state

    The drone rises to a specified hight, triggerd by user input (pressing space).
3. Explore / Path following

    The drone follows path by move base, triggered by height check.
4. Landing

    The drone descend to the ground, triggered by explore.
5. Landed

    The drone land to the ground and statys still, triggered by height check.


## state machine package

### Main Dependencies

- service_pkg
### quick launch

```bash
roslaunch state_machine_pkg state_machine.launch
```

then start controller, explore and press **Space** to trigger the state 2.

### Nodes:
- launch_keyboard_node (monitor user input)
- Quadrotor / Quadrotor2:
  - state_machine
### Topic tree:
- launch_keyboard (keyboard input)
- Quadrotor / Quadrotor2:
  - state_machine:
    - expected_height
    - state

### Services:
- Quadrotor / Quadrotor2:
  - state_machine:
    - stop_service (switch state from 3 to 4)
