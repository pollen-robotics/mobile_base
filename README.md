# Packages to control the mobile base Zuuu in ROS2
![Zuuu Drawing](./zuuu_drawing.png)
## Packages

* [zuuu_hal](./zuuu_hal/) - Hardware Abstraction Layer for the mobile base, ROSifies the inputs and outputs.
* [zuuu_interfaces](./zuuu_interfaces/) - Defines custom services for the mobile base
* [rplidar_ros](./rplidar_ros/) - RPLIDAR ROS2 Package.
* [mobile_base_sdk_server](./mobile_base_sdk_server/) - Hardware Abstraction Layer for the mobile base, ROSifies the inputs and outputs.


See the readme of each package for more information and specific installation needs.

## Installation

* Clone the repository into your ROS2 workspace
* Build the workspace with `colcon build --symlink-install`