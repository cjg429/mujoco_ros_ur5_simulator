# mujoco_ros_ur5_simulator

### Requirements

ROS Kinetic, conda, python 3.6

### Install ROS

Follow http://wiki.ros.org/kinetic/Installation/Ubuntu

build your catkin workspace with [catkin_ws](https://github.com/cjg429/mujoco_ros_ur5_simulator/tree/main/catkin_ws)

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-moveit
```

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
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules
```

