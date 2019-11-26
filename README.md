ros_msv
=========

This is a ROS 1 metapackage for controlling the *MSV_01* rescue robot. It provides full usage and monitoring of the electronic stage of the robot within ROS. The metapackage has eight packages, including a package for *ad-hoc* messages generation and a *bringup* package. The package has been tested on ROS Indigo but compatible with ROS Kinetic as well.

### Prerequisites

[ROS](http://www.ros.org) Indigo should be installed. The package also uses third party software such as:

 - The [liblightmodbus](https://github.com/Jacajack/liblightmodbus) lightweight, cross-platform Modbus library.
 - [HerkulexSDK](https://github.com/daconjurer/), a library for controlling the Hovis HerkuleX servo motors (Please see http://www.dstrobot.com/eng/index.jsp).

### Installing

Upon downloading the repo and placing it in the *src* folder of your *catkin_workspace*, on the *catkin_workspace* directory run the command

```
catkin_make
```

## Overview

The metapackage has two main packages for teleoperation of the *MSV_01* robot: ```msv_main``` and ```msv_teleop```. The ```msv_electric``` package retireves the sensing data. The ```msv_dxl```, ```msv_imu``` and ```msv_arm``` packages provide additional functionality (ROBOTIS Dynamixel AX-12A servos usage, IMU MPU6050 usage and the robotic arm control, respectively). The stack also includes a ```msv_bringup``` package for launching the nodes of the package and a ```msv_msgs``` package for *ad-hoc* messages generation.

## Getting started

Soon to become...

## Authors

* **Victor Sandoval** - [daconjurer](https://github.com/daconjurer)

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

* Both *msv_electric* and *msv_teleop* packages use the Modbus library liblightmodbus from **Jacek Wieczorek**.
* The *teleop* part is based on the *rescue* ROS 1 package from **José Armando Sanchez Rojas**.


