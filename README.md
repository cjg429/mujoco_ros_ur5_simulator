# mujoco_ros_ur5_simulator

### Requirements

ROS Kinetic, conda, python 3.6

### Install ROS

Follow http://wiki.ros.org/kinetic/Installation/Ubuntu

build your catkin workspace with [catkin_ws](https://github.com/cjg429/mujoco_ros_ur5_simulator/tree/main/catkin_ws)

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-moveit
```
- source /opt/ros/kinetic/setup.bash
- source ~/catkin_ws/devel/setup.bash

### Install Conda

Follow https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html

### Install MuJoCo

```
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3 libglew-dev
conda create -n mujoco python=3.6
conda activate mujoco
conda install -c anaconda patchelf 
```
Follow https://github.com/openai/mujoco-py

```
conda install -c conda-forge ros-rospy 
pip install scipy sympy opencv-python
```
Add following lines to bashrc file
- export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/jaegu/.mujoco/mujoco210/bin
- export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
- export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so:/usr/lib/nvidia-440/libGL.so

### Launch UR5 moveit package
```
roslaunch ur5_robotiq_moveit demo.launch
```
### Run ur5 planner 
```
python2 ur5_moveit/planner.py
```
### Run MuJoCo planner in conda envrionemnt
```
conda activate mujoco
python ur5_moveit/mujoco_planner.py
```
### Install GQCNN
Follow https://berkeleyautomation.github.io/gqcnn/install/install.html#pip-installation

depenecies: berkeley/autolab-core, berkeley/perception, berkeley/visualization

## Editing XML

https://ropiens.tistory.com/170
