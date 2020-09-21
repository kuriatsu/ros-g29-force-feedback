ros-g29-force-feedback
====

## Overview
Ros package to control force feedback of logitech g29 steering wheel from ros message, written in c++, for human beings all over the world.
This is useful for the user interface of autonomous driving, driving simulator like [CARLA](https://carla.org/), [LGSVL](https://www.lgsvlsimulator.com/) etc.

![logitech g29](https://github.com/kuriatsu/ros-g29-force-feedback/blob/image/images/logicoolg29.png)

## Features
* Standalone ros package to control steering wheel. (doesn't depend on the other ros packages like ros-melodic-joy etc.)

* We can control angle of logitech g29 steering wheel with throwing ros message.

* Two control modes
    1. PID control mode  
        Rotate wheel to the specified angle with PID control.(I controller is now deprecated). This mode can rotate wheel to the specified angle with yout hands off, so this mode is useful for the user interface of autonomous driving system. Control force can be specified (max force for PID control, the bigger, the more move rapidly).

    1. Constant force mode  
        Rotate wheel to the specified angle with specified constant force. This mode make it easier to control vehicle manually in the driving simulator. If you use with your hands off and rotate force over 0.3, the wheel travels right and left. 

## Demo
![demo_gif](https://github.com/kuriatsu/ros-g29-force-feedback/blob/image/images/force_feedback_test.gif)

## Requirement
* ubuntu
* ros melodic
* Logitech G29 Driving Force Racing Wheel

To check whether your kernel supports force feedback, do as follows
```bash
$ cat /boot/config-5.3.0-46-generic | grep CONFIG_LOGIWHEELS_FF
CONFIG_LOGIWHEELS_FF=y
```  
If you cannot get `CONFIG_LOGIWHEELS_FF=y`, try to find patch...


## Usage
1. confirm your device name
    ```bash
    $ cat /proc/bus/input/devices
    ```
    find **Logitech G29 Driving Force Racing Wheel** and check Handlers (ex. eventxx)

1. change parameters in **g29_force_feedback.yaml**
    |parameter|default|description|
    |:--|:--|:--|
    |device_name|/dev/input/event19|device name|
    |mode|0|control mode 0: PID control, 1: Auto centering
    |Kp|1|P value of PID contol|
    |Ki|0.0|I value of PID contol (Deprecated)|
    |Kd|0.1|D value of PID contol|
    |offset|0.01|affordable radian(offset * 2.5&pi;) of control|
    |max_force|1.0|max force|
    |min_force|0.2|0.25 is best! less than 0.2 cannot turn the wheel (in my case)|
    |pub_rate|0.1|event update rate (0.1=10Hz)|

1. run ros node
    ```bash
    $ source /path/to/catkin_ws/devel/setup.bash
    $ rosparam load /path/to/catkin_ws/src/g29_force_feedback/g29_force_feedback.yaml
    $ rosrun g29-force-feedback node
    ```

1. Throw message (It's better to use tab completion)  
    ```bash
    $ rostopic pub /ff_target g29-force-feedback/ForceFeedback "header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    angle: 0.3
    force: 0.6"
    ```
    Once the message is thrown, the wheel rotates to 0.3*2.5&pi; with 0.6 rotation power with PID control.
    Publish rate is not restricted.

## Install
1. create catkin_ws
    ```bash
    cd /path/to/any/dir
    mkdir -p catkin_ws/src
    cd /catkin_ws/src
    catkin_init_workspace
    ```
1. download package
    ```bash
    cd /catkin_ws/src
    git clone https://github.com/kuriatsu/g29-force-feedback.git
    cd ../
    catkin_make
    ```

## Contribution
1. Fork it (https://github.com/kuriatsu/g29-force-feedback.git)
1. Create your feature branch (git checkout -b my-new-feature)
1. Commit your changes (git commit -am 'Add some feature')
1. Push to the branch (git push origin my-new-feature)
1. Create new Pull Request

## Licence

[MIT](https://github.com/tcnksm/tool/blob/master/LICENCE)

## Author

[kuriatsu](https://github.com/kuriatsu)
