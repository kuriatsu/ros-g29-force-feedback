![GitHub forks](https://img.shields.io/github/forks/kuriatsu/ros-g29-force-feedback?style=flat&color=%#D78695)   ![GitHub Repo stars](https://img.shields.io/github/stars/kuriatsu/ros-g29-force-feedback?style=flat&color=%23FCCC06)  ![GitHub last commit](https://img.shields.io/github/last-commit/kuriatsu/ros-g29-force-feedback)    ![ros](https://img.shields.io/badge/ROS-Humble-blue)    ![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple)

# ros-g29-force-feedback
ROS2 package to control force feedback of logitech g29 steering wheel with ros message, written in c++, for human beings all over the world.
This is useful for the user interface of autonomous driving, driving simulator like [CARLA](https://carla.org/), [LGSVL](https://www.lgsvlsimulator.com/) etc.

`examples/carla_control.py` 

![demo_gif](https://github.com/kuriatsu/ros-g29-force-feedback/blob/image/images/force_feedback_test.gif)

# Features
* Standalone ros package to control steering wheel. (doesn't depend on the other ros packages like ros-melodic-joy etc.)
* Two control modes

    * Control mode (`config/g29.yaml/auto_centering=false`)  
    Rotate wheel to the specified angle (`position`) with specified `torque` as specified with rostopic.

    * Auto centering mode (`config/g29.yaml/auto_centering=true`)  
    Automatically centering position, without publishing rostopic.


* ROS1 and ROS2 support. (if you use ROS1, checkout and refer [ros1 branch](https://github.com/kuriatsu/ros-g29-force-feedback/tree/ros1))
    |ROS version|g29|g923|
    |:--|:--|:--|
    |ROS1|--|--|
    |Kinetic|tested|no|
    |Melodic|tested|no|
    |Noetic|tested|no|
    |ROS2|--|--|
    |Dashing|no|no|
    |Foxy|tested|no|
    |Galactic|tested|no|
    |Humble|no|no|

# Requirement of `master` branch
* ubuntu18-22
* ROS2
* Logitech G29 Driving Force Racing Wheel (Planning to test with g923)

To check whether your kernel supports force feedback, do as follows
```bash
$ cat /boot/config-5.3.0-46-generic | grep CONFIG_LOGIWHEELS_FF
CONFIG_LOGIWHEELS_FF=y
```  
If you cannot get `CONFIG_LOGIWHEELS_FF=y`, try to find patch or use latest kernel...

# Install
1. create ros2_ws
    ```bash
    mkdir -p ros2_ws/src
    cd /ros2_ws
    colcon build
    ```
2. download and build package
    ```bash
    cd /ros2_ws/src
    git clone https://github.com/kuriatsu/ros-g29-force-feedback.git
    cd ../
    colcon build --symlink-install
    ```
    
# Usage
1. Get device name
    ```bash
    $ cat /proc/bus/input/devices
    ```
    find **Logitech G29 Driving Force Racing Wheel** and check Handlers (ex. event19)

2. Change `device_name` in config/g29.yaml to the device name you obtained in step 1

3. Launch ros node
    ```bash
    $ source ros2_ws/install/setup.bash
    $ ros2 run ros_g29_force_feedback g29_force_feedback --ros-args --params-file ros2_ws/src/ros_g29_force_feedback/config/g29.yaml 
    ```

1. Publish message (It's better to use tab completion)  
    ```bash
    $ ros2 topic pub /ff_target ros_g29_force_feedback/msg/ForceFeedback "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, position: 0.3, torque: 0.5}"
    ```
    Once the message is published, the wheel rotates to 0.3*<max_angle> (g29: max_angle=450° clockwise, -450° counterclockwise).
    Publish rate is not restricted.
    
# Parameter

**g29_force_feedback.yaml**
|parameter|default|description|
|:--|:--|:--|
|device_name|/dev/input/event19|device name, change the number|
|loop_rate|0.1|Loop of retrieving wheel position and uploading control to the wheel|
|max_torque|1.0|As for g29, 1.0 = 2.5Nm (min_torque < max_torque < 1.0)|
|min_torque|0.2|Less than 0.2 cannot rotate wheel|
|brake_torque|0.2|Braking torque to stop at the position (descrived below)|
|brake_position|0.1|Brake angle (`position`-0.1*max_angle)|
|auto_centering_max_torque|0.3|Max torque for auto centering|
|auto_centering_max_position|0.2|Max torque position while auto centering (`position`±0.2*max_angle)|
|eps|0.01|Wheel in the range (position-eps to position+eps) is considered as it has reached the `position`|
|auto_centering|false|Anto centering if true|

![ros_g29_ff](https://user-images.githubusercontent.com/38074802/167057448-1fa21956-ae91-4e51-bee4-1fcdc05cae51.png)


# Contribution
1. Fork it (https://github.com/kuriatsu/g29-force-feedback.git)
1. Create your feature branch (git checkout -b my-new-feature)
1. Commit your changes (git commit -am 'Add some feature')
1. Push to the branch (git push origin my-new-feature)
1. Create new Pull Request


# Author

[kuriatsu](https://github.com/kuriatsu)

# Change Log

## 2023-03-19
Bag fix in ros1 branch thanks to [pedrohdsimoes](https://github.com/pedrohdsimoes)

## 2022-11-6
### examples added
An example script of CARLA connection was added. Rotate the wheel according to the ego vehicle spawned in CARLA as shown in the GIF image.

## 2022-04-21

### ROS2-Foxy integration
Now available in ROS2-Foxy thanks to [JLBicho](https://github.com/JLBicho)

## 2021-11-03
### Huge Improvement !!! 
Oscillation problem solved.
Wheel stops at the specified position, then starts auto centering.
Auto centering mode are set by rosparam in config/g29.yaml, not by rostopic.
Name of topic variables changed.

## 2020-10-10
### changed
PID-Constant mode can be changed dynamically!!
Removed mode selection from rosparam.
Rotation force is ignored when PID mode. (max force can be specified with rosparam (not dynamic))

# Reference
https://www.kernel.org/doc/html/v5.4/input/ff.html  
https://github.com/flosse/linuxconsole/tree/master
