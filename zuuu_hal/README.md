# Zuuu Hardware Abstraction Layer (HAL)

## Purpose

Zuuu HAL is responsible for interfacing with **Zuuu** (Reachy's mobile base) hardware and exposing its inputs/outputs through **ROS 2**.

- Periodically reads key measurements from the wheel controllers (speed, temperature, voltage, etc.).
- Publishes odometry data via `/odom` and the appropriate TF transform.
- Exposes multiple control interfaces, allowing:
  - Direct velocity commands (`x_vel, y_vel, theta_vel`).
  - Goal-based navigation (`x, y, theta` in the odometric frame).

See the [Usage](#usage) section for details.

---

## Installation

### Install PyVESC (Custom Version)
```sh
cd ~/dev
git clone https://github.com/pollen-robotics/PyVESC.git
pip3 install -e PyVESC/
```

### Install Dependencies
```sh
pip3 install transforms3d
```

For **ROS Foxy**:
```sh
sudo apt install ros-foxy-tf-transformations
```

For **ROS Humble**:
```sh
sudo apt install ros-humble-tf-transformations
```

---

## Dependencies

- **[zuuu_interfaces](../zuuu_interfaces):** Defines custom services for the mobile base.
- **[zuuu_description](../zuuu_description):** Required for visualization or simulation (e.g., RViz, Gazebo).  
  📌 Installation details can be found in the [dedicated README](../zuuu_description/README.md).

---

## Usage

### Running the HAL
```sh
ros2 launch zuuu_hal hal.launch.py
```

### Running the HAL with the Mobile Base SDK Server

If you prefer to use **Python commands** instead of ROS topics/services, you can run the **Mobile Base SDK server** alongside the HAL which is done automatically if you're using Reachy).



📌 [Then you can use the Mobile Base SDK Examples](https://github.com/pollen-robotics/reachy2-sdk/blob/develop/src/examples/5_mobile_base.ipynb)

---

## Controlling the Mobile Base

### Using a Controller
```sh
ros2 run zuuu_hal teleop_joy
```

### Using a Keyboard
```sh
ros2 run zuuu_hal teleop_keyboard
```

### Sending Velocity Commands
- Publish directly to the `/cmd_vel` topic.

### Using the **SetSpeed** Service
- Allows setting a constant speed for a specified duration.

Example (full rotation command):
```sh
ros2 service call /SetSpeed zuuu_interfaces/srv/SetSpeed "{x_vel: 0.0, y_vel: 0.0, rot_vel: 2.0, duration: 3.1415}"
```

Square movement test:
```sh
ros2 run zuuu_hal set_speed_service_test
```

### Using the **Goto** Action
- Moves the robot to a specific **(x, y, theta)** position in the odometric frame.

📌 See the [Goto Action](#goto-action) section for details.

### Setting the drive mode
CMD_GOTO is the default mode. Services will automatically change the drive mode as needed. 

:bulb: The most common use case where handling drive modes by hand is useful is when going back to CMD_GOTO (or CMD_VEL) is needed after a call to SetSpeed or GoToXYTheta.

1. CMD_VEL = The commands read on the topic /cmd_vel are applied after smoothing

2. BRAKE =  Sets the PWMs to 0 effectively braking the base

3. FREE_WHEEL =  Sets the current control to 0, coast mode

4. SPEED =  Mode used by the set_speed service to do speed control over arbitrary duration

5. GOTO =  Mode used by the go_to_xytheta service to do position control in odom frame

6. EMERGENCY_STOP =  Calls the emergency_shutdown method

7. CMD_GOTO =  Behaves like CMD_VEL but uses the odometry to correct the commands

Can be tested with CLI:
```
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: CMD_VEL}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: BRAKE}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: FREE_WHEEL}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: SPEED}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: GOTO}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: EMERGENCY_STOP}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: CMD_GOTO}" 
```

---

## Goto Action

The **Goto** action allows Zuuu to move to a specific goal position in the **odom** frame using two separate PID controllers:

- **Translation PID**
- **Rotation PID**

### Features:
✔ **Stackable** – Queue multiple goals.  
✔ **Monitorable** – Track progress in real-time.  
✔ **Cancelable** – Interrupt goals anytime.

### Example Usage:

#### **Using the ROS 2 Action Client**
📌 See [`zuuu_goto_action_client.py`](https://github.com/pollen-robotics/reachy2-sdk/blob/develop/src/zuuu_hal/zuuu_goto_action_client.py).

#### **Using a Jupyter Notebook Example**
📌 [Mobile Base SDK Example](https://github.com/pollen-robotics/reachy2-sdk/blob/develop/src/examples/5_mobile_base.ipynb).

#### **Running the Action Client Test**
```sh
ros2 run zuuu_hal goto_client_test
```

---

## Odometry

### Get Odometry
```sh
ros2 service call /GetOdometry zuuu_interfaces/srv/GetOdometry "{}"
```

### Reset Odometry
```sh
ros2 service call /ResetOdometry zuuu_interfaces/srv/ResetOdometry "{}"
```

⚠ **Safety Note:** If `ResetOdometry` is called while `GoToXYTheta` is running, it will automatically be **stopped**.

### Visual Test in RViz
```sh
ros2 launch zuuu_hal hal.launch.py
ros2 launch zuuu_description rviz_bringup.launch.py
```

Alternatively, launch the full system (HAL, LiDAR, RViz, and URDF model):
```sh
ros2 launch zuuu_description zuuu_bringup.launch.py
```

---

## Parameters

The parameter configuration file is located at:
```
config/params.yaml
```

Example: Dynamically changing the LiDAR angular limits:
```sh
ros2 param set /zuuu_hal laser_lower_angle -0.1
```

⚠ **Important:** The node **must** be run with its parameter file, or it will crash at launch (use `hal.launch.py` to ensure proper loading).

---

## Community & Support

💡 Learn more at **[pollen-robotics.com](https://pollen-robotics.com)**.  
💬 Join our **[Discord Community](https://discord.com/invite/vnYD6GAqJR)** to ask questions and share ideas.

