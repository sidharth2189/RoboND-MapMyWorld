# Map My World
The purpose of this repository is to create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the [RTAB-Map package](https://wiki.ros.org/rtabmap_ros). 

The steps are listed as [summary of tasks](task_summary.txt).

<img src="amcl_robot.gif"/>

## Description
Inside the Gazebo world one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .WhereAmI                               # Robot localization Project
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch folder for launch files   
    │   │   ├── robot_description.launch    # Generate urdf from xacro
    │   │   ├── world.launch                # launch Gazebo world along with robot
    │   │   ├── amcl.launch                 # launch robot localization using amcl
    │   │   ├── mapping.launch              # launch mapping node using rtabmap
    │   │   ├── localization.launch         # launch mapping with localization    
    │   ├── meshes                          # meshes folder for sensors
    │   │   ├── hokuyo.dae                  # Hokuyo lidar sensor
    │   ├── urdf                            # urdf folder for xarco files
    │   │   ├── my_robot.gazebo             # Plugin for sensor/actuator (RGBD camera/Hokuyo lidar/Differential drive)
    │   │   ├── my_robot.xacro              # Robot description
    │   ├── world                           # world folder for world files
    │   │   ├── office.world
    │   ├── CMakeLists.txt                  # compiler instructions
    │   ├── package.xml                     # package info
    │   ├── config                          # parmater for robot's navigational goal   
    │   │   ├── costmap_common_params.yaml  # rosparam for move_base package
    │   │   ├── local_costmap_params.yaml   # rosparam for move_base package
    │   │   ├── global_costmap_params.yaml  # rosparam for move_base package
    │   │   ├── base_costmap_params.yaml    # rosparam for move_base package
    │   ├── maps                            # parmater for robot's navigational goal   
    │   │   ├── map.pgm                     # map generated from pgm_map_creator package
    │   │   ├── map.yaml                    # map metadata
    │   ├── rtabmap                         # database generated from mapping
    │   │   ├── rtabmap.db                  # database file    
    ├── teleop_twist_keyboard               # package to control robot motion through keyboard (submodule)
    ├── amcl.rviz                           # visualization file for localization using amcl                           
    └──                          

### Dependencies

* Operating System — Ubuntu 16.04 LTS. ([Udacity VM Image](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip))
    *  Please refer steps for usage of VM, resource allocation and first boot [here](/docs/VM.txt).

### Installing
* To verify installation, run
```
gazebo
```

### How to generate map from gazebo world environment
* The ROS amcl node uses a [map file](/docs/pgm_map).
* This step helps generate a [map](/my_robot/maps/map.pgm) for the robot to knows what to expect in environment.
* For this purpose, [pgm_map_creator](/pgm_map_creator/) is used.
* Navigate to ROS package folder and create a maps folder. That's where the map file will reside.
```
cd /home/workspace/catkin_ws/src/<YOUR PACKAGE NAME>
```
```
mkdir maps
```
* Install Dependencies for compiling map creator.
```
sudo apt-get install libignition-math2-dev protobuf-compiler
```
* Clone repository or use the [submodule](/pgm_map_creator/) in this repository.
```
cd /home/workspace/catkin_ws/src/
```
```
git clone https://github.com/hyfan1116/pgm_map_creator.git
```
* Build package
```
cd ..
catkin_make
```
* Add and Edit the World File
Copy the Gazebo world you created to the world folder in pgm_map_creator
```
cp <YOUR GAZEBO WORLD FILE> src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
```
* Insert the map creator plugin to world file. Open the world file using the editor of your choice. Add the following tag towards the end of the file, but just before </world> tag:
```
<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
```
* Create the PGM map
Open a terminal, run gzerver with the map file
```
gzserver src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
```
* Open another terminal, launch the request_publisher node
```
roslaunch pgm_map_creator request_publisher.launch
```
* Wait for the plugin to generate map. It will be located in the map folder of the pgm_map_creator! 
* Open it to do a quick check of the map. If the map is cropped, you might want to adjust the parameters in launch/request_publisher.launch, namely the x and y values, which defines the size of the map
```
  <arg name="xmin" default="-15" />
  <arg name="xmax" default="15" />
  <arg name="ymin" default="-15" />
  <arg name="ymax" default="15" />
  <arg name="scan_height" default="5" />
  <arg name="resolution" default="0.01" />
```
* Edit the Map
If map is not accurate due to the models, feel free to edit the pgm file directly!

* Add the Map to robot Package
Now we have the map file, let us move it to where it is needed! That is the maps folder created at the very beginning of [robot package](/my_robot/)
```
cd /home/workspace/catkin_ws/
cp src/pgm_map_creator/maps/<YOUR MAP NAME>  src/<YOUR PACKAGE NAME>/maps/<YOUR MAP NAME>
```

* A yaml file providing the [metadata about the map](https://wiki.ros.org/map_server#YAML_format). Create a yaml file next to map.
```
cd src/<YOUR PACKAGE NAME>/src/maps
touch <YOUR MAP NAME>.yaml
```

* Add below lines to the yaml file.
```
image: <YOUR MAP NAME>
resolution: 0.01
origin: [-15.0, -15.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

* Note that the origin of the map should correspond to map's size. For example, the default map size is 30 by 30, so the origin will be [-15, -15, 0], i.e. half the size of the map.

### How to run localization
* Update and upgrade the Workspace
```
sudo apt-get update && sudo apt-get upgrade -y
```
* Create a [catkin workspace](https://wiki.ros.org/catkin/conceptual_overview)
```
$ mkdir -p ~/catkin_ws/src
```
* Navigate to source directory
```
$ cd ~/catkin_ws/src
```
* Initialize the catkin workspace which will create a ```CMakeLists.txt``` file.
```
catkin_init_workspace
```
* Clone this repository and its submodules.
```
git clone https://github.com/sidharth2189/RoboND-WhereAmI.git
```
```
git submodule update --init --recursive
```
* Copy ```my_robot```, ```pgm_map_creator``` and ```teleop_twist_keyboard``` packages into the source folder for catkin workspace.```/catkin_ws/src```
* Navigate to catkin workspace.
```
cd ~/catkin_ws/
```
* Build packages.Note that the command is issued from within the top level directory (i.e., within ```catkin_ws``` NOT ```catkin_ws/src```) 
```
catkin_make
```
* Source the set up script of the workspace. 
```
source devel/setup.bash
```
* To check for missing package.
```
rosdep check <package name>
```
* Launch the robot inside the world. Alongside Gazebo, this also open rviz for visualization.
```
roslaunch my_robot world.launch
```
* Launch amcl in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch my_robot amcl.launch
```
* To visualize the map and robot localization load [amcl.rviz](/amcl.rviz) using rviz window.

## Useful links
* [Oppeni Kinnect](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#OpenniKinect) 3D Camera description file.
[teleop node](https://github.com/ros-teleop/teleop_twist_keyboard) is used instead to send command for robot movement, using keyboard.
* [Robot reference](https://github.com/sidharth2189/RoboND-WhereAmI)
* [Graph slam](http://robot.cc/papers/thrun.graphslam.pdf) for large scale mapping of urban structures.
* [Occupancy grid mapping](https://wiki.ros.org/gmapping). The gmapping package provides laser-based SLAM. as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
* [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) ROS wrapper of [Real-Time Appearance-Based Mapping](http://introlab.github.io/rtabmap/). This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. 
* [rtabmap_viz](http://wiki.ros.org/rtabmap_viz). This node starts the visualization interface of RTAB-Map.
* [RTAB-Map 3D Lidar SLAM](https://www.youtube.com/watch?v=ytsfhMdv9W0)
* [RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation](https://arxiv.org/html/2403.06341v1)
* [Depth sensor cameras](https://www.e-consystems.com/blog/camera/technology/what-are-depth-sensing-cameras-how-do-they-work/?srsltid=AfmBOoq2aSWT2x0gB7iKADZXFQxWZhHf1KYBNXQfyo5Xnu2USkFZlyPb)
* [Octomap](https://wiki.ros.org/octomap)