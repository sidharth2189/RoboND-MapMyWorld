# Map My World
The purpose of this repository is to create a 2D occupancy grid and 3D octomap from a simulated environment using [robot](/my_robot/) with the [RTAB-Map](https://introlab.github.io/rtabmap/). 

The steps are listed as [summary of tasks](task_summary.txt).

<img src="MapMyWorld.gif"/>

## Description
Inside the Gazebo world one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .MapMyWorld                             # Robot localization Project
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch folder for launch files   
    │   │   ├── robot_description.launch    # Generate urdf from xacro
    │   │   ├── world.launch                # launch Gazebo world along with robot
    │   │   ├── amcl.launch                 # launch robot localization using amcl
    │   │   ├── mapping.launch              # launch mapping node using rtabmap
    │   │   ├── localization.launch         # launch mapping with localization
    │   │   ├── teleop.launch               # launch teleop_twist_keyboard to move robot    
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
    ├── pgm_map_creator                     # map creator package (submodule)    
    ├── teleop_twist_keyboard               # package to control robot motion through keyboard (submodule)
    ├── amcl.rviz                           # visualization file for localization using amcl                           
    └──                          

### Node view
<img src="/docs/rqt_graph.png"/>

### Dependencies

* Operating System — Ubuntu 16.04 LTS. ([Udacity VM Image](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip))
    *  Please refer steps for usage of VM, resource allocation and first boot [here](/docs/VM.txt).

### Installing
* To verify installation, run
```
gazebo
```

### How to run
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
git clone https://github.com/sidharth2189/RoboND-MapMyWorld.git
```
```
git submodule update --init --recursive
```
* Copy ```my_robot``` and [```teleop_twist_keyboard```](https://github.com/ros-teleop/teleop_twist_keyboard) packages into the source folder for catkin workspace.```/catkin_ws/src```
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
* Launch teleop in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch my_robot teleop.launch
```
* Launch localization using rtbmaps in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch my_robot localization.launch
```
  * Please note that, in the launch file, setting the parameter ```Mem/IncrementalMemory``` to ```false```, causes [empty working memory](/docs/warning_memory.png)
  * Hence, it is set to ```true```.
* Navigate your robot in the simulation to create map for the environment.
* When this is done, terminal the node and the map db file can be found in the place specified in the launch file. If the argument is not modified, it will be located in the /root/.ros/ folder.
* Database analysis: The ```rtabmap-databaseViewer``` is used for exploring database after generation. It is isolated from ROS and allows for complete analysis of your mapping session.
* Open map database as below
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
* Add some windows to get a better view of the relevant information.
  * Say yes to using the database parameters
  * View -> Constraint View
  * View -> Graph View
  * On the left, you have your 2D grid map in all of its updated iterations and the path of your robot.
  * In the middle you have different images from the mapping process. Here you can scrub through images to see all of the features from your detection algorithm. These features are in yellow. The pink indicates where two images have features in common and this information is being used to create neighboring links and loop closures!
  * Finally, on the right you can see the constraint view. This is where you can identify where and how the neighboring links and loop closures were created.
  * You can see the number of loop closures in the bottom left.
  * The codes stand for the following: Neighbor, Neighbor Merged, Global Loop closure, Local loop closure by space, Local loop closure by time, User loop closure, and Prior link.  

### Best practices
* One can start with lower velocities.
* The Goal is to create a great map with the least amount of passes as possible.
* Maximize loop closures by going over similar paths two or three times. 
* This allows for the maximization of feature detection, facilitating faster loop closures

## Useful links
* [Oppeni Kinnect](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#OpenniKinect) 3D Camera description file.
* [Ros Teleop package](https://github.com/ros-teleop/teleop_twist_keyboard) is used to send command for robot movement, using keyboard.
* [Robot reference](https://github.com/sidharth2189/RoboND-WhereAmI)
* [Graph slam](http://robot.cc/papers/thrun.graphslam.pdf) for large scale mapping of urban structures.
* [Occupancy grid mapping](https://wiki.ros.org/gmapping). The gmapping package provides laser-based SLAM. as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
* [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)- It is the ROS wrapper of [Real-Time Appearance-Based Mapping](http://introlab.github.io/rtabmap/). This package can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. 
* [rtabmap_viz](http://wiki.ros.org/rtabmap_viz)- This node starts the visualization interface of RTAB-Map.
* [RTAB-Map 3D Lidar SLAM](https://www.youtube.com/watch?v=ytsfhMdv9W0)
* [RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation](https://arxiv.org/html/2403.06341v1)
* [Depth sensor cameras](https://www.e-consystems.com/blog/camera/technology/what-are-depth-sensing-cameras-how-do-they-work/?srsltid=AfmBOoq2aSWT2x0gB7iKADZXFQxWZhHf1KYBNXQfyo5Xnu2USkFZlyPb)
* [Octomap](https://octomap.github.io/)
* [Tutorial on graph based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)
* [Tether-Aware Path Planning for Autonomous Exploration of Unknown Environments](https://www.youtube.com/watch?v=nROO0BFK4lc)