# RO47014 template repository 

This is a commit message by developer from Group 13

This repository contains information regarding the simulation and other packages used for the course on Knowledge Representation and Reasoning RO47014 to be given in Q3 21-22 by Carlos Hernández Corbato.

## Installation

### (recommended) install using singularity image
**Please follow the instructions in the practicum 1.2 PDF**

### (optional) install locally
You can install the TIAGo simulation and our code with the following steps
First, open a terminal, create an empty workspace and clone this repository:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitlab.tudelft.nl/cor/ro47014/2022_course_projects/group_00/retail_store_skills.git
```
### From source

Use vcs to clone the additionally required repositories:

```bash
vcs import < retail_store_skills/retail_store_skills.repos
```

>*Note*: The command above requires [vcstool](https://github.com/dirk-thomas/vcstool), if you do not have it installed you can install it with the command:
`sudo apt install python3-vcstool`

Set up **rosdep**
```bash
sudo rosdep init
rosdep update
```

Then you may run the following instruction to make sure that all dependencies referenced in the workspace are installed
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3"
```

Before building, run this to make sure you have all the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) dependencies
```bash
sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet
```
Finally, build the workspace
```bash
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0
source devel/setup.bash
```
From this point on it's all ready to go.

