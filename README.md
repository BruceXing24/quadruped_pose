## Quadruped_pose

### Introduce:
This is a quadruped pose controller using robot kinematic. This can be used in position control motor quadruped

### Environment:
ubuntu = 22.04 \
ros2-humble \
numpy = 1.22.4 \
pybullet = 3.2.5 \

### Calculation 
robot kinematic refer to []
pose calculation refer to []

### Run in ros
cd <workspace/src>
git clone https://github.com/FurqanHabibi/joystick_ros2 \
git clone https://github.com/BruceXing24/quadruped_pose.git \
but there is a problem when build joystick, in jointstick_ros2, line 189, there is a parameter Qos missing here.
![avatar](https://github.com/BruceXing24/quadruped_pose/blob/master/media/1.jpg)

  #### joystick
  ![avatar-w300](https://github.com/BruceXing24/quadruped_pose/blob/master/media/joysticker.jpg){zoom:50%}
  
  #### video
  video can be found in []
  here show servek gif 

