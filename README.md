# mujoco_ros_ur5_simulator

### Requirements

ROS Kinetic, conda, python 3.6

### Install ROS

Follow http://wiki.ros.org/kinetic/Installation/Ubuntu

build your catkin workspace with [catkin_ws]((https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat)](https://openai.github.io/mujoco-py/build/html/index.html)

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


