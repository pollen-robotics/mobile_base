# SDK Server for Reachy's Mobile Base

ROS2 package:
* using the topics and services defined in the [mobile base's HAL](../zuuu_hal) to get mobile base information (e.g. ododmetry, battery level, control mode, drive mode, ...) and send mobility commands,
* handling mobile base grpc services defined in [mobile_platform_reachy.proto](https://github.com/pollen-robotics/reachy-sdk-api/blob/main/protos/mobile_platform_reachy.proto).
 
**ROS2 Version: Humble**

Dependencies: [reachy_sdk_api](https://github.com/pollen-robotics/reachy-sdk-api),
[zuuu_interfaces](https://github.com/pollen-robotics/zuuu_interfaces),
[grpcio](https://pypi.org/project/grpcio/),
[grpcio-tools](https://pypi.org/project/grpcio-tools/),
[protobuf](https://pypi.org/project/protobuf/).


## GRPC services handled
* **SendDirection** - Send velocities commands to the mobile base for a predifined period of time.
* **SendSetSpeed** - Send velocities commands to the mobile base.
* **SendGoTo** - Send GoTo instruction to the mobile base.
* **DistanceToGoal** - Return delta x, delta y, delta theta and distance from the last goal position sent using SendGoTo.
* **SetControlMode** - Set the mobile base's control mode.
* **GetControlMode** - Get the mobile base's control mode.
* **SetZuuuMode** - Set the mobile base's drive mode.
* **GetZuuuMode** - Get the mobile base's drive mode.
* **GetBatteryLevel** - Get the mobile base's battery voltage.
* **GetOdometry** - Get the mobile base's odometry.
* **ResetOdometry** - Reset the mobile base's odometry.
* **SetZuuuSafety** - Disable / enable the mobile base's anti-collision safety provided by the Lidar.
* **GetMobileBasePresence** - Return if a mobile base is specified in Reachy's config file. If yes, return the mobile base's version.

## Launch files
* **mobile_base_sdk_server.launch.py** - Launch mobile_base_sdk_server node.
* **run_mobile_base_sdk_server_and_hal.launch.py** - Launch the mobile base's HAL nodes and the mobile_base_sdk_server node.

---

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or join our [Dicord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your ideas.
