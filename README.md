# TerraSentia navigation package
A package with everything you need to run traversability algorithms on TerraSentia robots

## Setup

This repository is created to run on a custom modification of the TerraSentia robot.

![outline](images/terrasentia_new.png)

Our TerraSentia mod, referred as TerraSentia*, contains a Velodyne Puck VLP-16 LiDAR and three fixed RGB cameras.

### To use DLIO as the state estimator

Clone the DLIO package (our fork):
```shell
git clone https://github.com/matval/direct_lidar_inertial_odometry
```
### Run the navigation package
- Make sure the package is compiled. In your ROS workspace, run:
```shell
catkin make
```
Then, run the navigation launch
```shell
roslaunch terrasentia_navigation terrasentia_navigation.launch
```