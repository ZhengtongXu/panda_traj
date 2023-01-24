# Real-time trajectory following for Franka Panda
## Prerequisites
- Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
- catkin_simple (https://github.com/catkin/catkin_simple)
- track_ik (https://traclabs.com/projects/trac-ik/)
- franka_ros (https://github.com/frankaemika/franka_ros)
- NLopt (https://nlopt.readthedocs.io/en/latest/)
## Build
```bash
$ cd mv panda_traj ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make
```
## Usage
```bash
$ roslaunch simple_franka_interface real_time_traj_following.launch
$ rosrun traj_generator example
```
Any position controller, velocity controller, and motion planner generating end-effector motion commands can be integrated into this package. See `example.cpp` for more information.
