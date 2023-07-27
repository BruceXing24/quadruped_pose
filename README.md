## Quadruped_pose

### Introduce:
This is a quadruped pose controller using robot kinematic and PID. This can be used in position control motor quadruped

### Environment:
ubuntu = 22.04 \
ros2-humble \
numpy = 1.22.4 \
pybullet = 3.2.5 \

### Calculation 
robot kinematic refer to https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
pose calculation refer to (https://zhuanlan.zhihu.com/p/64321561)


### Run in ros
cd <workspace/src>
git clone https://github.com/FurqanHabibi/joystick_ros2 \
git clone https://github.com/BruceXing24/quadruped_pose.git \
but there is a problem when build joystick, in jointstick_ros2, line 189, there is a parameter Qos missing here.
![avatar](https://github.com/BruceXing24/quadruped_pose/blob/master/media/1.jpg)

  #### joystick
  <img src="https://github.com/BruceXing24/quadruped_pose/blob/master/media/joysticker.jpg" width="500px">
  
  #### result show
  video can be found in https://www.bilibili.com/video/BV1HG4y157Ct/ \
  here show servel gif 

