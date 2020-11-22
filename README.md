# EUFS Autonomous Simulation

ROS/Gazebo simulation packages for driverless FSAE vehicles.
This repository is a hard fork of www.github.com/eufsa/eufs_sim repository, forked by ASURT team of Ain Shams University, Egypt

### Contents
1. [Install Prerequisites](#requirements)
2. [Compiling and running](#compiling)
3. [Sensors](#sensors)

## Setup Instructions
### 1. Install Prerequisites <a name="requirements"></a>
##### - Install Ubuntu 20.04 LTS
##### - Install [ros-noetic-desktop-full](http://wiki.ros.org/noetic/Installation)
##### - Install ROS packages:
```
sudo apt-get install ros-noetic-ackermann-msgs ros-noetic-twist-mux ros-noetic-joy ros-noetic-controller-manager ros-noetic-robotnik-msgs ros-noetic-velodyne-simulator ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-joint-state-controller ros-noetic-gazebo-ros-control
```


### 2. Compiling and running <a name="compiling"></a>

Clone this workspace in your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/asurt-fsai/eufs_sim.git
```
Copy the contents of this repository to the `src` folder you just created.

Navigate to your workspace and build the simulation:
```
cd ~/catkin_ws
catkin_make OR catkin build
```
_Note:_ You can use `catkin build` instead of `catkin_make` if you know what you are doing.

To enable ROS to find the EUFS packages you also need to run
```source /devel/setup.bash```
_Note:_ source needs to be run on each new terminal you open. You can also include it in your `.bashrc` file.

Now you can finally run eufs simulation!!
```
roslaunch eufs_gazebo small_track.launch
roslaunch eufs_gazebo skidpad.launch
roslaunch eufs_gazebo acceleration.launch
roslaunch eufs_gazebo big_track.launch
```


An easy way to control the car is via
```roslaunch robot_control rqt_robot_control.launch```

**Sensor suit of the car by default:**

* VLP16 lidar
* ZED Stereo camera
* IMU
* GPS
* odometry

**We at Ain Shams University giving a big thank you to EUFS team**
