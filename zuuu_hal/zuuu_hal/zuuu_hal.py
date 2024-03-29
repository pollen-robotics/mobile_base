"""
    Zuuu's Hardware Abstraction Layer main node.
    'Hardware' here means the three wheel controllers, the battery and the LIDAR.
    The responsability of the node is to read the sensors, handle common calculations 
    (filtering, odometry, inverse kinematics) and exposing control interfaces.

    The following ROS services are exposed to:
    - Get or reset the odometry (GetOdometry, ResetOdometry)
    - Get the battery voltage (GetBatteryVoltage)
    - Set an (x_vel, y_vel, theta_vel) speed command for a fixed period of time (SetSpeed)
    - Set a (x, y, theta) goal position in the odometric frame (GoToXYTheta, IsGoToFinished, DistanceToGoal)
    - Manage drive modes (GetZuuuMode, SetZuuuMode)
    - Enable/disable the LIDAR safety (SetZuuuSafety)

    The following topics are published for:
    - Filtering the LIDAR (scan_filterd)
    - Reading the wheels rotational speed (back_wheel_rpm, left_wheel_rpm, right_wheel_rpm)
    - Reading the odometry (odom)

    See params.yaml for the list of ROS parameters.
"""

# - Safety lidar image

import copy
import math
import os
import sys
import time
import traceback
from collections import deque
from csv import writer
from enum import Enum
from subprocess import check_output
from typing import List

import numpy as np
import rclpy
import rclpy.logging
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from pyvesc.VESC import MultiVESC
from rcl_interfaces.msg import SetParametersResult
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from reachy_utils.config import ReachyConfig
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from zuuu_interfaces.srv import (
    DistanceToGoal,
    GetBatteryVoltage,
    GetOdometry,
    GetZuuuMode,
    GetZuuuSafety,
    GoToXYTheta,
    IsGoToFinished,
    ResetOdometry,
    SetSpeed,
    SetZuuuMode,
    SetZuuuSafety,
)

from zuuu_hal.lidar_safety import LidarSafety
from zuuu_hal.utils import PID, angle_diff, sign


class ZuuuModes(Enum):
    """
    Zuuu drive modes
    CMD_VEL = The commands read on the topic /cmd_vel are applied after smoothing
    BRAKE =  Sets the PWMs to 0 effectively braking the base
    FREE_WHEEL =  Sets the current control to 0, coast mode
    SPEED =  Mode used by the set_speed service to do speed control over arbitrary duration
    GOTO =  Mode used by the go_to_xytheta service to do position control in odom frame
    EMERGENCY_STOP =  Calls the emergency_shutdown method
    """

    CMD_VEL = 1
    BRAKE = 2
    FREE_WHEEL = 3
    SPEED = 4
    GOTO = 5
    EMERGENCY_STOP = 6
    CMD_GOTO = 7


class ZuuuControlModes(Enum):
    """
    Zuuu control modes
    OPEN_LOOP = The HAL will send PWM commands to the controllers.
    PID = The HAL will send speed commands to the controllers, the control is made by the low level PIDs
    """

    OPEN_LOOP = 1
    PID = 2


class MobileBase:
    """Mobile base representation and the interface with low level controllers"""

    def __init__(
        self,
        serial_port: str = "/dev/vesc_wheels",
        left_wheel_id: int = 24,
        right_wheel_id: int = 72,
        back_wheel_id: int = 116,
    ) -> None:
        params = [
            {"can_id": left_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": right_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": back_wheel_id, "has_sensor": True, "start_heartbeat": True},
        ]
        self._multi_vesc = MultiVESC(serial_port=serial_port, vescs_params=params)

        self.left_wheel, self.right_wheel, self.back_wheel = (
            self._multi_vesc.controllers
        )
        (
            self.left_wheel_measurements,
            self.right_wheel_measurements,
            self.back_wheel_measurements,
        ) = (None, None, None)
        self.left_wheel_nones, self.right_wheel_nones, self.back_wheel_nones = 0, 0, 0
        self.wheel_radius = 0.21 / 2.0
        self.wheel_to_center = 0.19588  # 0.188
        self.half_poles = 15.0
        self.left_wheel_rpm, self.right_wheel_rpm, self.back_wheel_rpm = 0, 0, 0
        self.left_wheel_avg_rpm, self.right_wheel_avg_rpm, self.back_wheel_avg_rpm = (
            0,
            0,
            0,
        )
        (
            self.left_wheel_rpm_deque,
            self.right_wheel_rpm_deque,
            self.back_wheel_rpm_deque,
        ) = (
            deque([], 10),
            deque([], 10),
            deque([], 10),
        )
        # These values might be a tad too safe, however the battery should be almost empty when the cells are
        # on average at 3.3V so there is little to win to go below this. Still tunable if needed.
        # The current battery has a BMS that shuts down the battery at 20V +-1V. So that would be 2.86V +-0.14V.
        self.battery_cell_warn_voltage = 3.5
        self.battery_cell_min_voltage = 3.3
        self.battery_nb_cells = 7
        self.battery_check_period = 60

    def read_all_measurements(self) -> None:
        """Reads all the measurements for the left, right and back wheels"""
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()
        self.back_wheel_measurements = self.back_wheel.get_measurements()

    def deque_to_avg(self, deque: deque) -> float:
        """Returns the average of the values contained in deque"""
        sum = 0
        len = deque.maxlen
        for i in deque:
            sum += i
        return sum / float(len)


class ZuuuHAL(Node):
    """Zuuu's Hardware Abstraction Layer node"""

    def __init__(self) -> None:
        """Node initialisation.
        ROS side: setup of the timers, callbacks, topics, services and parameters.
        Low level side: connecting with the hardware and making a first read of the sensors
        """
        super().__init__("zuuu_hal")
        self.get_logger().info("Starting zuuu_hal!")
        # self.zuuu_model = check_output(
        #     os.path.expanduser('~')+'/.local/bin/reachy-identify-zuuu-model'
        #     ).strip().decode()

        reachy_config = ReachyConfig()
        self.zuuu_version = reachy_config.mobile_base_config["version_hard"]
        self.get_logger().info(f"zuuu version: {self.zuuu_version}")
        try:
            float_model = float(self.zuuu_version)
            if float_model < 1.0:
                self.omnibase = MobileBase(
                    left_wheel_id=24, right_wheel_id=72, back_wheel_id=None
                )
            elif float_model < 1.2:
                self.omnibase = MobileBase(
                    left_wheel_id=24, right_wheel_id=None, back_wheel_id=116
                )
            else:
                self.omnibase = MobileBase(
                    left_wheel_id=None, right_wheel_id=72, back_wheel_id=116
                )
        except Exception:
            msg = (
                "ZUUU version can't be processed, check that the 'zuuu_version' tag is "
                "present in the .reachy.yaml file"
            )
            self.get_logger().error(msg)
            self.get_logger().error(traceback.format_exc())
            raise RuntimeError(msg)

        self.get_logger().info("Reading Zuuu's sensors once...")
        self.read_measurements()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("laser_upper_angle", 2.85),
                ("laser_lower_angle", -2.85),
                ("max_duty_cyle", 0.20),
                ("cmd_vel_timeout", 0.2),
                ("max_full_com_fails", 100),
                ("main_tick_period", 0.012),
                ("control_mode", "OPEN_LOOP"),
                ("max_accel_xy", 1.0),
                ("max_accel_theta", 1.0),
                ("max_speed_xy", 0.5),
                ("max_speed_theta", 2.0),
                ("xy_tol", 0.0),
                ("theta_tol", 0.0),
                ("smoothing_factor", 5.0),
                ("safety_distance", 0.70),
                ("critical_distance", 0.55),
            ],
        )
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Maing sure we don't run the node if the config file is not shared
        if self.get_parameter("max_duty_cyle").type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'max_duty_cyle' was not initialized. Check that the param file is given to the node"
                "(using the launch file is the way to go). Shutting down"
            )
            self.destroy_node()
        if self.get_parameter("smoothing_factor").type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'smoothing_factor' was not initialized. Check that the param file is given to the node"
                "(using the launch file is the way to go). Shutting down"
            )
            self.destroy_node()

        # Parameters initialisation
        self.laser_upper_angle = (
            self.get_parameter("laser_upper_angle").get_parameter_value().double_value
        )  # math.pi
        self.laser_lower_angle = (
            self.get_parameter("laser_lower_angle").get_parameter_value().double_value
        )  # -math.pi
        self.max_duty_cyle = (
            self.get_parameter("max_duty_cyle").get_parameter_value().double_value
        )  # 0.3  # max is 1
        self.cmd_vel_timeout = (
            self.get_parameter("cmd_vel_timeout").get_parameter_value().double_value
        )  # 0.2
        self.max_full_com_fails = (
            self.get_parameter("max_full_com_fails").get_parameter_value().integer_value
        )  # 100
        self.main_tick_period = (
            self.get_parameter("main_tick_period").get_parameter_value().double_value
        )  # 0.012

        control_mode_param = self.get_parameter("control_mode")
        if control_mode_param.value in [m.name for m in ZuuuControlModes]:
            # "OPEN_LOOP"
            self.control_mode = ZuuuControlModes[control_mode_param.value]
        else:
            self.get_logger().error(
                f"Parameter 'control_mode' has an unknown value: '{control_mode_param.value}'. Shutting down"
            )
            self.destroy_node()

        self.max_accel_xy = (
            self.get_parameter("max_accel_xy").get_parameter_value().double_value
        )  # 1.0
        self.max_accel_theta = (
            self.get_parameter("max_accel_theta").get_parameter_value().double_value
        )  # 1.0
        self.max_speed_xy = (
            self.get_parameter("max_speed_xy").get_parameter_value().double_value
        )  # 0.5
        self.max_speed_theta = (
            self.get_parameter("max_speed_theta").get_parameter_value().double_value
        )  # 2.0
        self.xy_tol = (
            self.get_parameter("xy_tol").get_parameter_value().double_value
        )  # 0.2
        self.theta_tol = (
            self.get_parameter("theta_tol").get_parameter_value().double_value
        )  # 0.17
        self.smoothing_factor = (
            self.get_parameter("smoothing_factor").get_parameter_value().double_value
        )  # 100.0
        self.safety_distance = (
            self.get_parameter("safety_distance").get_parameter_value().double_value
        )

        self.critical_distance = (
            self.get_parameter("critical_distance").get_parameter_value().double_value
        )

        self.cmd_vel = None
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0
        self.x_vel_goal = 0.0
        self.y_vel_goal = 0.0
        self.theta_vel_goal = 0.0
        self.x_vel_goal_filtered = 0.0
        self.y_vel_goal_filtered = 0.0
        self.theta_vel_goal_filtered = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.reset_odom = False
        self.battery_voltage = 25.0
        self.mode = ZuuuModes.CMD_GOTO
        self.speed_service_deadline = 0
        self.speed_service_on = False
        self.goto_service_on = False
        self.safety_on = False
        # scan_is_read==True is a temporary fix to allow the stack to run without the LIDAR
        self.scan_is_read = True
        self.scan_timeout = 0.5
        self.nb_control_ticks = 0
        self.stationary_on = False
        self.lidar_safety = LidarSafety(
            self.safety_distance,
            self.critical_distance,
            robot_collision_radius=0.5,
            speed_reduction_factor=0.88,
            logger=self.get_logger(),
        )
        self.cv_bridge = CvBridge()

        self.x_pid = PID(p=3.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0)
        self.y_pid = PID(p=3.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0)
        self.theta_pid = PID(
            p=1.0, i=0.0, d=0.00, max_command=4.0, max_i_contribution=1.0
        )
        # "No over-shoot Ziegler Nichols"
        # self.theta_pid = PID(p=3.2, i=14.2, d=0.475,
        #                      max_command=4.0, max_i_contribution=1.0)

        self.max_wheel_speed = self.pwm_to_wheel_rot_speed(self.max_duty_cyle)
        self.get_logger().info(
            f"The maximum PWM value is {self.max_duty_cyle*100}% => maximum wheel speed is set to "
            f"{self.max_wheel_speed:.2f}rad/s"
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.cmd_vel_sub  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_filter_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.scan_sub  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME
        self.scan_pub = self.create_publisher(LaserScan, "scan_filterd", 10)
        self.lidar_image_pub = self.create_publisher(Image, "lidar_image", 1)

        self.pub_back_wheel_rpm = self.create_publisher(Float32, "back_wheel_rpm", 2)
        self.pub_left_wheel_rpm = self.create_publisher(Float32, "left_wheel_rpm", 2)
        self.pub_right_wheel_rpm = self.create_publisher(Float32, "right_wheel_rpm", 2)

        self.pub_odom = self.create_publisher(Odometry, "odom", 2)

        self.mode_service = self.create_service(
            SetZuuuMode, "SetZuuuMode", self.handle_zuuu_mode
        )

        self.get_mode_service = self.create_service(
            GetZuuuMode, "GetZuuuMode", self.handle_get_zuuu_mode
        )

        self.reset_odometry_service = self.create_service(
            ResetOdometry, "ResetOdometry", self.handle_reset_odometry
        )

        self.get_odometry_service = self.create_service(
            GetOdometry, "GetOdometry", self.handle_get_odometry
        )

        self.set_speed_service = self.create_service(
            SetSpeed, "SetSpeed", self.handle_set_speed
        )

        # I chose not to make an action client. Could be changed if needed.
        self.go_to_service = self.create_service(
            GoToXYTheta, "GoToXYTheta", self.handle_go_to
        )

        self.is_go_to_finished = self.create_service(
            IsGoToFinished, "IsGoToFinished", self.handle_is_go_to_finished
        )

        self.distance_to_goal = self.create_service(
            DistanceToGoal, "DistanceToGoal", self.handle_distance_to_goal
        )

        self.get_battery_voltage_service = self.create_service(
            GetBatteryVoltage, "GetBatteryVoltage", self.handle_get_battery_voltage
        )

        self.set_safety_service = self.create_service(
            SetZuuuSafety, "SetZuuuSafety", self.handle_zuuu_set_safety
        )

        self.get_safety_service = self.create_service(
            GetZuuuSafety, "GetZuuuSafety", self.handle_zuuu_get_safety
        )

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.old_measure_timestamp = self.get_clock().now()
        self.measure_timestamp = self.get_clock().now()
        # *sigh* if needed use: https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py
        self.cmd_vel_t0 = time.time()
        self.scan_t0 = time.time()
        self.t0 = time.time()
        self.read_measurements()
        self.first_tick = True
        self.only_x = True
        self.theta_null = True
        self.joy_angle = 0.0
        self.joy_intesity = 0.0
        self.joy_rotation_on = False
        self.save_odom_checkpoint_xy()
        self.save_odom_checkpoint_theta()
        self.dir_p0 = [0.0, 0.0]
        self.dir_p1 = [1.0, 0.0]
        self.dir_angle = 0.0

        self.create_timer(self.main_tick_period, self.main_tick)
        # self.create_timer(0.1, self.main_tick)
        self.measurements_t = time.time()
        # Checking battery once at the start, then periodically
        self.check_battery()
        self.create_timer(self.omnibase.battery_check_period, self.check_battery)

    def parameters_callback(self, params) -> None:
        """When a ROS parameter is changed, this method will be called to verify the change and accept/deny it."""
        success = False
        for param in params:
            if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                if param.name == "laser_upper_angle":
                    self.laser_upper_angle = param.value
                    success = True
                elif param.name == "laser_lower_angle":
                    self.laser_lower_angle = param.value
                    success = True
                elif param.name == "max_duty_cyle":
                    if param.value >= 0.0 and param.value <= 1.0:
                        self.max_duty_cyle = param.value
                        success = True
                elif param.name == "cmd_vel_timeout":
                    if param.value >= 0.0:
                        self.cmd_vel_timeout = param.value
                        success = True
                elif param.name == "max_full_com_fails":
                    if param.value >= 0.0:
                        self.max_full_com_fails = param.value
                        success = True
                elif param.name == "main_tick_period":
                    if param.value >= 0.0:
                        self.main_tick_period = param.value
                        success = True
                elif param.name == "max_accel_xy":
                    if param.value >= 0.0:
                        self.max_accel_xy = param.value
                        success = True
                elif param.name == "max_accel_theta":
                    if param.value >= 0.0:
                        self.max_accel_theta = param.value
                        success = True
                elif param.name == "max_speed_xy":
                    if param.value >= 0.0:
                        self.max_speed_xy = param.value
                        success = True
                elif param.name == "max_speed_theta":
                    if param.value >= 0.0:
                        self.max_speed_theta = param.value
                        success = True
                elif param.name == "xy_tol":
                    if param.value >= 0.0:
                        self.xy_tol = param.value
                        success = True
                elif param.name == "theta_tol":
                    if param.value >= 0.0:
                        self.theta_tol = param.value
                        success = True
                elif param.name == "smoothing_factor":
                    if param.value >= 0.0:
                        self.smoothing_factor = param.value
                        success = True
                elif param.name == "safety_distance":
                    if param.value >= 0.0:
                        self.safety_distance = param.value
                        success = True
                elif param.name == "critical_distance":
                    if param.value >= 0.0:
                        self.critical_distance = param.value
                        success = True

            elif param.type_ is Parameter.Type.STRING:
                if param.name == "control_mode":
                    if param.value in [m.name for m in ZuuuControlModes]:
                        self.control_mode = ZuuuControlModes[param.value]
                        success = True

        return SetParametersResult(successful=success)

    def handle_zuuu_mode(
        self, request: SetZuuuMode.Request, response: SetZuuuMode.Response
    ) -> SetZuuuMode.Response:
        """Handle SetZuuuMode service request"""
        self.get_logger().info("Requested mode change to :'{}'".format(request.mode))
        response.success = False

        if request.mode in [m.name for m in ZuuuModes]:
            if request.mode == ZuuuModes.SPEED.name:
                self.get_logger().info(
                    "'{}' should not be changed by hand, use the SetSpeed service instead".format(
                        request.mode
                    )
                )
            elif request.mode == ZuuuModes.GOTO.name:
                self.get_logger().info(
                    "'{}' should not be changed by hand, use the GoToXYTheta service instead".format(
                        request.mode
                    )
                )
            else:
                # Changing the mode is a way to prematurely end an on going task requested through a service
                self.stop_ongoing_services()
                self.mode = ZuuuModes[request.mode]
                response.success = True
                self.get_logger().info("OK")

        return response

    def handle_get_zuuu_mode(
        self, request: GetZuuuMode.Request, response: GetZuuuMode.Response
    ) -> GetZuuuMode.Response:
        """Handle GetZuuuMode service request"""
        response.mode = self.mode.name
        return response

    def handle_reset_odometry(
        self, request: ResetOdometry.Request, response: ResetOdometry.Response
    ) -> ResetOdometry.Response:
        """Handle ResetOdometry service request"""
        # Resetting asynchronously to prevent race conditions.
        self.reset_odom = True
        self.get_logger().info("Requested to reset the odometry frame")
        response.success = True
        return response

    def handle_get_odometry(
        self, request: GetOdometry.Request, response: GetOdometry.Response
    ) -> GetOdometry.Response:
        response.x = self.x_odom
        response.y = self.y_odom
        response.theta = self.theta_odom
        return response

    def handle_set_speed(
        self, request: SetSpeed.Request, response: SetSpeed.Response
    ) -> SetSpeed.Response:
        """Handle SetSpeed service request"""
        # This service automatically changes the zuuu mode
        self.mode = ZuuuModes.SPEED
        self.get_logger().info(
            f"Requested set_speed: duration={request.duration} x_vel='{request.x_vel}'m/s, y_vel='{request.y_vel}'m/s,"
            f"rot_vel='{request.rot_vel}'rad/s"
        )
        self.x_vel_goal = request.x_vel
        self.y_vel_goal = request.y_vel
        self.theta_vel_goal = request.rot_vel
        self.speed_service_deadline = time.time() + request.duration
        self.speed_service_on = True
        response.success = True
        return response

    def handle_go_to(
        self, request: GoToXYTheta.Request, response: GoToXYTheta.Response
    ) -> GoToXYTheta.Response:
        """Handle GoToXYTheta service request"""
        # This service automatically changes the zuuu mode
        self.mode = ZuuuModes.GOTO
        self.get_logger().info(
            f"Requested go_to: x={request.x_goal}m, y={request.y_goal}m, theta={request.theta_goal}rad"
        )
        self.x_goal = request.x_goal
        self.y_goal = request.y_goal
        self.theta_goal = request.theta_goal
        self.goto_service_on = True
        self.x_pid.set_goal(self.x_goal)
        self.y_pid.set_goal(self.y_goal)
        self.theta_pid.set_goal(self.theta_goal)
        response.success = True
        return response

    def handle_is_go_to_finished(
        self, request: IsGoToFinished.Request, response: IsGoToFinished.Response
    ) -> IsGoToFinished.Response:
        """handle IsGoToFinished service request"""
        # Returns True if the goal position is reached
        response.success = self.goto_service_on
        return response

    def handle_distance_to_goal(
        self, request: DistanceToGoal.Request, response: DistanceToGoal.Response
    ) -> DistanceToGoal.Response:
        """Handle DistanceToGoal service resquest"""
        response.delta_x = self.x_goal - self.x_odom
        response.delta_y = self.y_goal - self.y_odom
        response.delta_theta = angle_diff(self.theta_goal, self.theta_odom)
        response.distance = math.sqrt(
            (self.x_goal - self.x_odom) ** 2 + (self.y_goal - self.y_odom) ** 2
        )
        return response

    def handle_get_battery_voltage(
        self, request: GetBatteryVoltage.Request, response: GetBatteryVoltage.Response
    ) -> GetBatteryVoltage.Response:
        """Handle GetBatteryVoltage service request"""
        response.voltage = self.battery_voltage
        return response

    def handle_zuuu_set_safety(
        self, request: SetZuuuSafety.Request, response: SetZuuuSafety.Response
    ) -> SetZuuuSafety.Response:
        """Handle SetZuuuSafety service request"""
        safety_on = request.safety_on
        state = "ON" if safety_on else "OFF"
        self.get_logger().info(f"Lidar safety is now {state}")
        self.safety_on = safety_on
        self.lidar_safety.safety_distance = request.safety_distance
        self.lidar_safety.critical_distance = request.critical_distance
        response.success = True
        return response

    def handle_zuuu_get_safety(
        self, request: GetZuuuSafety.Request, response: GetZuuuSafety.Response
    ) -> GetZuuuSafety.Response:
        """Handle GetZuuuSafety service request"""
        response.safety_on = self.safety_on
        response.safety_distance = self.lidar_safety.safety_distance
        response.critical_distance = self.lidar_safety.critical_distance
        response.obstacle_detection_status = self.lidar_safety.obstacle_detection_status
        return response

    def check_battery(self, verbose: bool = False) -> None:
        """Checks that the battery readings are not too old and forces a read if need be.
        Checks that the battery voltages are safe and warns or stops the HAL accordingly.
        """
        t = time.time()
        if verbose:
            self.print_all_measurements()
        if (t - self.measurements_t) > (self.omnibase.battery_check_period + 1):
            self.get_logger().warning(
                "Zuuu's measurements are not made often enough. Reading now."
            )
            self.read_measurements()
        warn_voltage = (
            self.omnibase.battery_nb_cells * self.omnibase.battery_cell_warn_voltage
        )
        min_voltage = (
            self.omnibase.battery_nb_cells * self.omnibase.battery_cell_min_voltage
        )
        voltage = self.battery_voltage

        if min_voltage < voltage < warn_voltage:
            self.get_logger().warning(
                "Battery voltage LOW ({}V). Consider recharging. Warning threshold: {:.1f}V, "
                "stop threshold: {:.1f}V".format(voltage, warn_voltage, min_voltage)
            )
        elif voltage < min_voltage:
            msg = "Battery voltage critically LOW ({}V). Emergency shutdown! Warning threshold: {:.1f}V, "
            "stop threshold: {:.1f}V".format(voltage, warn_voltage, min_voltage)
            self.get_logger().error(msg)
            self.emergency_shutdown()
            raise RuntimeError(msg)
        else:
            self.get_logger().warning("Battery voltage OK ({}V)".format(voltage))

    def emergency_shutdown(self) -> None:
        """Sets a PWM of 0V to the three wheel motors, equivalent to a brake."""
        self.omnibase.back_wheel.set_duty_cycle(0)
        self.omnibase.left_wheel.set_duty_cycle(0)
        self.omnibase.right_wheel.set_duty_cycle(0)
        self.get_logger().warn("Emergency shutdown!")
        time.sleep(0.1)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handles the callback on the /cmd_vel topic"""
        self.cmd_vel = msg
        self.cmd_vel_t0 = time.time()

    def scan_filter_callback(self, msg: LaserScan) -> None:
        """Callback method on the /scan topic. Handles the LIDAR filtering and safety calculations."""
        self.scan_is_read = True
        self.scan_t0 = time.time()
        # LIDAR angle filter managemnt
        # Temporary fix since https://github.com/ros-perception/laser_filters doesn't work on Foxy at the moment
        filtered_scan = LaserScan()
        filtered_scan.header = copy.deepcopy(msg.header)
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        ranges = []
        intensities = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle > self.laser_upper_angle or angle < self.laser_lower_angle:
                ranges.append(0.0)
                intensities.append(0.0)
            else:
                ranges.append(r)
                intensities.append(msg.intensities[i])

        filtered_scan.ranges = ranges
        filtered_scan.intensities = intensities
        self.scan_pub.publish(filtered_scan)

        # LIDAR safety management
        self.lidar_safety.clear_measures()
        if self.safety_on:
            self.lidar_safety.process_scan(filtered_scan)
        # Publishing the safety image
        lidar_img = self.lidar_safety.create_safety_img(filtered_scan)
        self.lidar_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(lidar_img))

    def wheel_rot_speed_to_pwm_no_friction(self, rot: float) -> float:
        """Uses a simple linear model to map the expected rotational speed of the wheel to a constant PWM
        (based on measures made on a full Reachy Mobile)
        """
        return rot / 22.7

    def wheel_rot_speed_to_pwm(self, rot: float) -> float:
        """Uses a simple affine model to map the expected rotational speed of the wheel to a constant PWM
        (based on measures made on a full Reachy Mobile)
        """
        # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
        epsilon = 0.02
        if rot > epsilon:
            pwm = 0.0418 * rot + 0.0126
        elif rot < -epsilon:
            pwm = 0.0418 * rot - 0.0126
        else:
            pwm = 0.0

        return pwm

    def pwm_to_wheel_rot_speed(self, pwm: float) -> float:
        """Uses a simple affine model to map a pwm to the expected rotational speed of the wheel
        (based on measures made on a full Reachy Mobile)
        """
        # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
        if abs(pwm) < 0.0126:
            rot = 0.0
        else:
            rot = sign(pwm) * (abs(pwm) - 0.0126) / 0.0418

        return rot

    def ik_vel_to_pwm(self, x_vel: float, y_vel: float, rot_vel: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame)
        and outputs the PWM to apply to each of the 3 motors in an omni setup

        Args:
            x (float): x speed (m/s). Positive "in front" of the robot.
            y (float): y speed (m/s). Positive "to the left" of the robot.
            rot (float): rotational speed (rad/s). Positive counter-clock wise.

        """
        rot_vels = self.ik_vel(x_vel, y_vel, rot_vel)
        return [self.wheel_rot_speed_to_pwm(rot_vel) for rot_vel in rot_vels]

    def ik_vel_old(self, x: float, y: float, rot: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame)
        and outputs the PWM to apply to each of the 3 motors in an omni setup

        Args:
            x (float): x speed (between -1 and 1). Positive "in front" of the robot.
            y (float): y speed (between -1 and 1). Positive "to the left" of the robot.
            rot (float): rotational speed (between -1 and 1). Positive counter-clock wise.
        """
        cycle_back = -y + rot
        cycle_right = (
            (-y * np.cos(120 * np.pi / 180)) + (x * np.sin(120 * np.pi / 180)) + rot
        )
        cycle_left = (
            (-y * np.cos(240 * np.pi / 180)) + (x * np.sin(240 * np.pi / 180)) + rot
        )

        return [cycle_back, cycle_right, cycle_left]

    # Interesting stuff can be found in Modern Robotics' chapters:
    # 13.2 Omnidirectional Wheeled Mobile Robots
    # 13.4 Odometry
    # /!\ Our robot frame is different. Matching between their names (left) and ours (right):
    # xb=y, yb=-x, theta=-theta, u1=uB, u2=uL, u3=uR
    def ik_vel(self, x_vel: float, y_vel: float, rot_vel: float) -> List[float]:
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame) and outputs the rotational speed (rad/s)
        of each of the 3 motors in an omni setup

        Args:
            x (float): x speed (m/s). Positive "in front" of the robot.
            y (float): y speed (m/s). Positive "to the left" of the robot.
            rot (float): rotational speed (rad/s). Positive counter-clock wise.
        """

        wheel_rot_speed_back = (1 / self.omnibase.wheel_radius) * (
            self.omnibase.wheel_to_center * rot_vel - y_vel
        )
        wheel_rot_speed_right = (1 / self.omnibase.wheel_radius) * (
            self.omnibase.wheel_to_center * rot_vel
            + y_vel / 2.0
            + math.sin(math.pi / 3) * x_vel
        )
        wheel_rot_speed_left = (1 / self.omnibase.wheel_radius) * (
            self.omnibase.wheel_to_center * rot_vel
            + math.sin(math.pi / 3) * y_vel / 2
            - math.sin(math.pi / 3) * x_vel
        )

        return [wheel_rot_speed_back, wheel_rot_speed_right, wheel_rot_speed_left]

    def dk_vel(self, rot_l: float, rot_r: float, rot_b: float) -> float:
        """Takes the 3 rotational speeds (in rpm) of the 3 wheels and outputs the x linear speed (m/s),
        y linear speed (m/s) and rotational speed (rad/s) in the robot egocentric frame

        Args:
            rot_l (float): rpm speed of the left wheel
            rot_r (float): rpm speed of the right wheel
            rot_b (float): rpm speed of the back wheel
        """
        # rpm to rad/s then m/s
        speed_l = (2 * math.pi * rot_l / 60) * self.omnibase.wheel_radius
        speed_r = (2 * math.pi * rot_r / 60) * self.omnibase.wheel_radius
        speed_b = (2 * math.pi * rot_b / 60) * self.omnibase.wheel_radius

        x_vel = -speed_l * (1 / (2 * math.sin(math.pi / 3))) + speed_r * (
            1 / (2 * math.sin(math.pi / 3))
        )
        y_vel = -speed_b * 2 / 3.0 + speed_l * 1 / 3.0 + speed_r * 1 / 3.0
        theta_vel = (speed_l + speed_r + speed_b) / (3 * self.omnibase.wheel_to_center)

        return [x_vel, y_vel, theta_vel]

    def filter_speed_goals(self):
        """Applies a smoothing filter on x_vel_goal, y_vel_goal and theta_vel_goal"""
        self.x_vel_goal_filtered = (
            self.x_vel_goal + self.smoothing_factor * self.x_vel_goal_filtered
        ) / (1 + self.smoothing_factor)
        self.y_vel_goal_filtered = (
            self.y_vel_goal + self.smoothing_factor * self.y_vel_goal_filtered
        ) / (1 + self.smoothing_factor)
        self.theta_vel_goal_filtered = (
            self.theta_vel_goal + self.smoothing_factor * self.theta_vel_goal_filtered
        ) / (1 + self.smoothing_factor)

        (
            self.x_vel_goal_filtered,
            self.y_vel_goal_filtered,
            self.theta_vel_goal_filtered,
        ) = self.lidar_safety.safety_check_speed_command(
            self.x_vel_goal_filtered,
            self.y_vel_goal_filtered,
            self.theta_vel_goal_filtered,
        )

    def format_measurements(self, measurements) -> str:
        """Text formatting for the low level controller measurements"""
        if measurements is None:
            return "None"
        to_print = ""
        to_print += "temp_fet:{}\n".format(measurements.temp_fet)
        to_print += "temp_motor:{}\n".format(measurements.temp_motor)
        to_print += "avg_motor_current:{}\n".format(measurements.avg_motor_current)
        to_print += "avg_input_current:{}\n".format(measurements.avg_input_current)
        to_print += "avg_id:{}\n".format(measurements.avg_id)
        to_print += "avg_iq:{}\n".format(measurements.avg_iq)
        to_print += "duty_cycle_now:{}\n".format(measurements.duty_cycle_now)
        to_print += "rpm:{}\n".format(measurements.rpm)
        to_print += "v_in:{}\n".format(measurements.v_in)
        to_print += "amp_hours:{}\n".format(measurements.amp_hours)
        to_print += "amp_hours_charged:{}\n".format(measurements.amp_hours_charged)
        to_print += "watt_hours:{}\n".format(measurements.watt_hours)
        to_print += "watt_hours_charged:{}\n".format(measurements.watt_hours_charged)
        to_print += "tachometer:{}\n".format(measurements.tachometer)
        to_print += "tachometer_abs:{}\n".format(measurements.tachometer_abs)
        to_print += "mc_fault_code:{}\n".format(measurements.mc_fault_code)
        to_print += "pid_pos_now:{}\n".format(measurements.pid_pos_now)
        to_print += "app_controller_id:{}\n".format(measurements.app_controller_id)
        to_print += "time_ms:{}\n".format(measurements.time_ms)
        return to_print

    def print_all_measurements(self) -> None:
        """Prints the low level measurements from the 3 wheel controllers"""
        to_print = "\n*** back_wheel measurements:\n"
        to_print += self.format_measurements(self.omnibase.back_wheel_measurements)
        to_print += "\n\n*** left_wheel:\n"
        to_print += self.format_measurements(self.omnibase.left_wheel_measurements)
        to_print += "\n\n*** right_wheel:\n"
        to_print += self.format_measurements(self.omnibase.right_wheel_measurements)
        to_print += "\n\n Fails ('Nones') left:{}, right:{}, back:{}".format(
            self.omnibase.left_wheel_nones,
            self.omnibase.right_wheel_nones,
            self.omnibase.back_wheel_nones,
        )
        to_print += "\n\n AVG RPM left:{:.2f}, right:{:.2f}, back:{:.2f}".format(
            self.omnibase.left_wheel_avg_rpm / self.omnibase.half_poles,
            self.omnibase.right_wheel_avg_rpm / self.omnibase.half_poles,
            self.omnibase.back_wheel_avg_rpm / self.omnibase.half_poles,
        )

        self.get_logger().info("{}".format(to_print))
        # 20 tours en 35s, avg_rpm ~=34

    def publish_wheel_speeds(self) -> None:
        """Publishes the most recent measure of rotational speed for each of the 3 wheels on 3 separate topics."""
        # If the measurements are None, not publishing
        if self.omnibase.back_wheel_measurements is not None:
            rpm_back = Float32()
            rpm_back.data = float(self.omnibase.back_wheel_measurements.rpm)
            self.pub_back_wheel_rpm.publish(rpm_back)

        if self.omnibase.left_wheel_measurements is not None:
            rpm_left = Float32()
            rpm_left.data = float(self.omnibase.left_wheel_measurements.rpm)
            self.pub_left_wheel_rpm.publish(rpm_left)

        if self.omnibase.right_wheel_measurements is not None:
            rpm_right = Float32()
            rpm_right.data = float(self.omnibase.right_wheel_measurements.rpm)
            self.pub_right_wheel_rpm.publish(rpm_right)

    def update_wheel_speeds(self) -> None:
        """Uses the latest mesure of wheel rotational speed to update the smoothed internal estimation of the wheel
        rotational speed
        """
        # Keeping a local value of the wheel speeds to handle None measurements (we'll use the last valid measure)
        if self.omnibase.back_wheel_measurements is not None:
            value = float(self.omnibase.back_wheel_measurements.rpm)
            self.omnibase.back_wheel_rpm = value
            self.omnibase.back_wheel_rpm_deque.appendleft(value)
            self.omnibase.back_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.back_wheel_rpm_deque
            )
        else:
            self.omnibase.back_wheel_nones += 1

        if self.omnibase.left_wheel_measurements is not None:
            value = float(self.omnibase.left_wheel_measurements.rpm)
            self.omnibase.left_wheel_rpm = value
            self.omnibase.left_wheel_rpm_deque.appendleft(value)
            self.omnibase.left_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.left_wheel_rpm_deque
            )
        else:
            self.omnibase.left_wheel_nones += 1

        if self.omnibase.right_wheel_measurements is not None:
            value = float(self.omnibase.right_wheel_measurements.rpm)
            self.omnibase.right_wheel_rpm = value
            self.omnibase.right_wheel_rpm_deque.appendleft(value)
            self.omnibase.right_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.right_wheel_rpm_deque
            )
        else:
            self.omnibase.right_wheel_nones += 1

    def publish_odometry_and_tf(self) -> None:
        """Publishes the current odometry position (Odometry type published on the /odom topic) and also
        published the TransformStamped between the frame base_footprint and odom
        """
        # Odom
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.measure_timestamp.to_msg()
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x_odom
        odom.pose.pose.position.y = self.y_odom
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_odom)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # TODO tune these numbers if needed
        odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        odom.twist.twist.linear.x = self.x_vel
        odom.twist.twist.linear.y = self.y_vel
        odom.twist.twist.angular.z = self.theta_vel

        odom.twist.covariance = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        self.pub_odom.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = self.measure_timestamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x_odom
        t.transform.translation.y = self.y_odom
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def tick_odom(self) -> None:
        """Updates the odometry values based on the small displacement measured since the last tick,
        then published the results with publish_odometry_and_tf()
        """
        # TODO VESC speed values are, as is normal, very noisy at low speeds.
        # We have no control on how the speeds are calculated as is.
        # -> By reading the encoder ticks directly and making the calculations here we could make this a tad better.

        # Local speeds in egocentric frame.
        # "rpm" are actually erpm and need to be divided by half the amount of magnetic poles to get the actual rpm.
        self.x_vel, self.y_vel, self.theta_vel = self.dk_vel(
            self.omnibase.left_wheel_rpm / self.omnibase.half_poles,
            self.omnibase.right_wheel_rpm / self.omnibase.half_poles,
            self.omnibase.back_wheel_rpm / self.omnibase.half_poles,
        )
        # self.get_logger().info(
        #     "IK vel : {:.2f}, {:.2f}, {:.2f}".format(self.x_vel, self.y_vel, self.theta_vel))
        # Applying the small displacement in the world-fixed odom frame (simple 2D rotation)
        dt_duration = self.measure_timestamp - self.old_measure_timestamp
        dt_seconds = dt_duration.nanoseconds / S_TO_NS
        dx = (
            self.x_vel * math.cos(self.theta_odom)
            - self.y_vel * math.sin(self.theta_odom)
        ) * dt_seconds
        dy = (
            self.x_vel * math.sin(self.theta_odom)
            + self.y_vel * math.cos(self.theta_odom)
        ) * dt_seconds
        dtheta = self.theta_vel * dt_seconds
        self.x_odom += dx
        self.y_odom += dy
        self.theta_odom += dtheta

        self.vx = dx / dt_seconds
        self.vy = dy / dt_seconds
        self.vtheta = dtheta / dt_seconds
        if self.reset_odom:
            # Resetting asynchronously to prevent race conditions.
            # dx, dy and dteta remain correct even on the reset tick
            self.reset_odom = False
            self.reset_odom_now()

            # Resetting the odometry while a GoTo is ON might be dangerous. Stopping it to make sure:
            if self.goto_service_on:
                self.goto_service_on = False
        self.publish_odometry_and_tf()

    def reset_odom_now(self):
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

    def limit_duty_cycles(self, duty_cycles: List[float]) -> List[float]:
        """Limits the duty cycles to stay in +-max_duty_cyle"""
        for i in range(len(duty_cycles)):
            if duty_cycles[i] < 0:
                temp = max(-self.max_duty_cyle, duty_cycles[i])
                if temp != duty_cycles[i]:
                    self.get_logger().warning("Duty cycle is LIMITED")
                duty_cycles[i] = temp
            else:
                temp = min(self.max_duty_cyle, duty_cycles[i])
                if temp != duty_cycles[i]:
                    self.get_logger().warning("Duty cycle is LIMITED")
                duty_cycles[i] = temp

        return duty_cycles

    def limit_wheel_speeds(self, wheel_speeds: List[float]) -> List[float]:
        """Limits the wheel speeds to stay in +-max_wheel_speed"""
        for i in range(len(wheel_speeds)):
            if wheel_speeds[i] < 0:
                temp = max(-self.max_wheel_speed, wheel_speeds[i])
                if temp != wheel_speeds[i]:
                    self.get_logger().warning("Wheel speed is LIMITED")
                wheel_speeds[i] = temp
            else:
                temp = min(self.max_wheel_speed, wheel_speeds[i])
                if temp != wheel_speeds[i]:
                    self.get_logger().warning("Wheel speed is LIMITED")
                wheel_speeds[i] = temp
        return wheel_speeds

    def limit_vel_commands(self, x_vel, y_vel, theta_vel):
        xy_speed = math.sqrt(x_vel**2 + y_vel**2)
        if xy_speed > self.max_speed_xy:
            # This formula guarantees that the ratio x_vel/y_vel remains the same , while ensuring the xy_speed is equal to max_speed_xy
            new_x_vel = math.sqrt(self.max_speed_xy**2 / (1 + (y_vel**2) / (x_vel**2)))
            new_y_vel = new_x_vel * y_vel / x_vel
            # self.get_logger().warning(
            #     f"Requesting xy_speed ({xy_speed}) above maximum ({self.max_speed_xy}). Reducing it to {math.sqrt(new_x_vel**2+new_y_vel**2)}")
            # The formula can mess up the signs, fixing them here
            x_vel = sign(x_vel) * new_x_vel / sign(new_x_vel)
            y_vel = sign(y_vel) * new_y_vel / sign(new_y_vel)
        if abs(theta_vel) > self.max_speed_theta:
            theta_vel = sign(theta_vel) * self.max_speed_theta
            self.get_logger().warning(
                f"Requesting theta_speed ({theta_vel}) above maximum ({self.max_speed_theta}). Reducing it."
            )

        return x_vel, y_vel, theta_vel

    def read_measurements(self) -> None:
        """Calls the low level functions to read the measurements on the 3 wheel controllers"""
        self.omnibase.read_all_measurements()
        if self.omnibase.back_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.back_wheel_measurements.v_in
        elif self.omnibase.left_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.left_wheel_measurements.v_in
        elif self.omnibase.right_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.right_wheel_measurements.v_in
        else:
            # Decidemment ! Keeping last valid measure...
            self.nb_full_com_fails += 1
            # self.get_logger().warning(
            #     "Could not read any of the motor drivers. This should not happen too often.")
            if self.nb_full_com_fails > self.max_full_com_fails:
                msg = "Too many communication errors, emergency shutdown"
                self.get_logger().error(msg)
                self.emergency_shutdown()
                raise RuntimeError(msg)
            return
        # Read success
        self.nb_full_com_fails = 0
        self.measurements_t = time.time()

    def send_wheel_commands(self, wheel_speeds: List[float]) -> None:
        """Sends either a PWM command or a speed command to the wheel controllers, based on the current control mode"""
        if self.control_mode is ZuuuControlModes.OPEN_LOOP:
            duty_cycles = [
                self.wheel_rot_speed_to_pwm(wheel_speed) for wheel_speed in wheel_speeds
            ]
            duty_cycles = self.limit_duty_cycles(duty_cycles)
            self.omnibase.back_wheel.set_duty_cycle(duty_cycles[0])
            self.omnibase.left_wheel.set_duty_cycle(duty_cycles[2])
            self.omnibase.right_wheel.set_duty_cycle(duty_cycles[1])
        elif self.control_mode is ZuuuControlModes.PID:
            # rad/s to rpm to erpm
            wheel_speeds = self.limit_wheel_speeds(wheel_speeds)
            self.omnibase.back_wheel.set_rpm(
                int(self.omnibase.half_poles * wheel_speeds[0] * 30 / math.pi)
            )
            self.omnibase.left_wheel.set_rpm(
                int(self.omnibase.half_poles * wheel_speeds[2] * 30 / math.pi)
            )
            self.omnibase.right_wheel.set_rpm(
                int(self.omnibase.half_poles * wheel_speeds[1] * 30 / math.pi)
            )
        else:
            self.get_logger().warning(
                "unknown control mode '{}'".format(self.control_mode)
            )

    def position_control(self) -> List[float]:
        """Calculates the speed targets to be sent on x, y and theta to reach their respective goals during a GoTo
        This function uses 3 separate PIDs controllers.
        """
        x_command_odom = self.x_pid.tick(self.x_odom)
        y_command_odom = self.y_pid.tick(self.y_odom)
        theta_command_odom = self.theta_pid.tick(self.theta_odom, is_angle=True)
        # self.get_logger().warning(f"theta error: '{self.theta_pid.prev_error:.2f}', command:'{theta_command_odom:.2f}'")

        x_command = x_command_odom * math.cos(
            -self.theta_odom
        ) - y_command_odom * math.sin(-self.theta_odom)
        y_command = x_command_odom * math.sin(
            -self.theta_odom
        ) + y_command_odom * math.cos(-self.theta_odom)

        return x_command, y_command, theta_command_odom

    def stop_ongoing_services(self) -> None:
        """Stops the GoTo and the SetSpeed services, if they were running"""
        self.goto_service_on = False
        self.speed_service_on = False

    def did_mode_change(self, dx, dy, dtheta, almost_zero=0.001):
        if self.only_x:
            if abs(dy) > almost_zero:
                self.only_x = False
                return True
        else:
            if abs(dx) > almost_zero:
                self.only_x = True
                return True
        return False

    def handle_joy_discretization(
        self, dx, dy, dtheta, almost_zero=0.001, nb_directions=8
    ):
        if abs(dx) < almost_zero and abs(dy) < almost_zero:
            rotation_on = False
            angle = 0
            intesity = 0
            is_stationary = True
        else:
            is_stationary = False
            joy_angle = math.atan2(dy, dx)
            intesity = math.sqrt(dx**2 + dy**2)
            angle_step = math.pi * 2 / nb_directions
            half_angle_step = angle_step / 2
            found = False
            for i in range(nb_directions):
                angle = i * angle_step
                if abs(angle_diff(angle, joy_angle)) <= half_angle_step:
                    # Found the discretization angle
                    found = True
                    break
            if not found:
                msg = "Impossible case in the joy discretization angle function, stopping for safety"
                self.get_logger().error(msg)
                self.emergency_shutdown()
                raise RuntimeError(msg)

        if abs(dtheta) < almost_zero:
            rotation_on = False
        else:
            rotation_on = True
        if self.joy_angle != angle:
            direction_changed = True
        else:
            direction_changed = False

        if self.joy_rotation_on != rotation_on:
            rotation_changed = True
        else:
            rotation_changed = False
        self.joy_angle = angle
        self.joy_intesity = intesity
        self.joy_rotation_on = rotation_on

        return (
            angle,
            intesity,
            rotation_on,
            direction_changed,
            rotation_changed,
            is_stationary,
        )

    def save_odom_checkpoint(self):
        self.save_odom_checkpoint_theta()
        self.save_odom_checkpoint_xy()

    def save_odom_checkpoint_xy(self):
        self.x_odom_checkpoint = self.x_odom
        self.y_odom_checkpoint = self.y_odom
        # self.get_logger().info(f"XY checkpoint")

    def save_odom_checkpoint_theta(self):
        self.theta_odom_checkpoint = self.theta_odom
        # self.get_logger().info(f"Theta checkpoint")

    def save_direction_checkpoint(self, angle):
        # Saving 2 points to save the unit vector of motion and its line
        self.dir_p0 = [self.x_odom, self.y_odom]
        self.dir_p1 = [self.x_odom + math.cos(angle), self.y_odom + math.sin(angle)]
        self.dir_angle = angle
        # self.get_logger().info(f"self.dir_p0={self.dir_p0}")
        # self.get_logger().info(f"self.dir_p1={self.dir_p1}")
        # self.get_logger().info(f"self.dir_angle={self.dir_angle}")

    def calculate_xy_goal(self, dist):
        # First, calulate the projection point p from the current robot position (based on the odometry) onto the line of motion.
        rx = self.x_odom
        ry = self.y_odom
        p0x = self.dir_p0[0]
        p0y = self.dir_p0[1]
        p1x = self.dir_p1[0]
        p1y = self.dir_p1[1]
        v_motion = [p1x - p0x, p1y - p0y]
        v_motion /= np.linalg.norm(v_motion, 2)
        v_robot = [rx - p0x, ry - p0y]
        p = self.dir_p0 + v_motion * np.dot(v_robot, v_motion)
        # Then add a translation of dist, along the line of motion
        p_goal = p + v_motion * dist
        return p_goal[0], p_goal[1]

    def fake_vel_goals_to_goto_goals(self, x_vel_goal, y_vel_goal, theta_vel_goal):
        # TODO : tune this (linear)
        dx = self.x_vel_goal_filtered
        dy = self.y_vel_goal_filtered
        dtheta = self.theta_vel_goal_filtered
        # Limiting the span of dtheta (if dtheta is allowed beyond PI, the PID will inverse the sign of the rotation, creating a discontinuity in control)
        max_dtheta = 5 * math.pi / 6
        if dtheta > max_dtheta:
            dtheta = max_dtheta
        elif dtheta < -max_dtheta:
            dtheta = -max_dtheta
        almost_zero = 0.001
        nb_control_ticks_wait = 10
        control_goals_updated = True
        # OK il faut faire du goal_theta quand on rotate pas, et du current sinon.
        (
            joy_angle,
            intensity,
            rotation_on,
            direction_changed,
            rotation_changed,
            is_stationary,
        ) = self.handle_joy_discretization(
            dx, dy, dtheta, almost_zero=almost_zero, nb_directions=8
        )

        # Checking if we need to update our reference points
        if rotation_changed:
            # Went from ON to OFF or from OFF to ON
            self.save_odom_checkpoint_theta()

        # The theta control is independent from x and y
        if rotation_on:
            # Applying the joy command (almost no disturbance rejection since the goal is relative to the odometric theta)
            self.theta_goal = self.theta_odom + dtheta
            # Updating the reference line of motion in odom frame, since a rotation happened and the position of the joy doesn't mean what it used to mean
            # When saving a direction based on a reference, there is a non trivial choice between choosing the current orientation (self.theta_odom)
            # and the goal orientation (self.theta_goal). Here we choose where we 'currently are' since the scene is currently rotating and the pilot would
            # most likely base his decision on the current view :
            joy_angle_odom_frame = joy_angle + self.theta_odom
            self.save_direction_checkpoint(joy_angle_odom_frame)
        else:
            # Controling to reach the stable goal position (strong disturbance rejection)
            self.theta_goal = self.theta_odom_checkpoint

        if direction_changed:
            # Saving the reference point in odom frame
            self.save_odom_checkpoint_xy()
            # Saving the reference line of motion in odom frame

            # Here, we save based on where we want to be (self.theta_goal) to increase the disturbance rejection in rotations
            joy_angle_odom_frame = joy_angle + self.theta_goal
            self.save_direction_checkpoint(joy_angle_odom_frame)

        if is_stationary:
            # Staying where we currently are in XY, letting theta do its thing
            if not self.stationary_on:
                self.stationary_on = True
                self.save_odom_checkpoint_xy()
            if not (rotation_on):
                # Fully static. Reducing the P to remove the oscillations created by the "steps" of the wheels
                self.theta_pid = PID(
                    p=0.5, i=0.0, d=0.00, max_command=4.0, max_i_contribution=1.0
                )
                self.x_pid = PID(
                    p=1.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0
                )
                self.y_pid = PID(
                    p=1.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0
                )
            self.x_goal = self.x_odom_checkpoint
            self.y_goal = self.y_odom_checkpoint
        else:
            if self.stationary_on:
                self.stationary_on = False
                self.save_odom_checkpoint_xy()
                # Here, we save based on where we want to be (self.theta_goal) to increase the disturbance rejection in rotations
                joy_angle_odom_frame = joy_angle + self.theta_goal
                self.save_direction_checkpoint(joy_angle_odom_frame)
            # The X and Y goals will always be on the reference line of motion
            # How far on the line ? Let's call P the projection point from the current robot position (based on the odometry) onto the line of motion.
            # The goal position will be 'dist' (e.g. how much the joy was pressed) away from P, on the line of motion.
            # This is the generalization of the simple case "the only motion is along X, let's set the y_pid goal to 0" to an arbitrary direction of motion.
            self.x_goal, self.y_goal = self.calculate_xy_goal(intensity)
            # self.get_logger().info(
            #     f"self.x_goal={self.x_goal:.2f}, self.y_goal={self.y_goal:.2f}, self.theta_goal={self.theta_goal:.2f}")

        # # V0 no smart correction, everything is open
        # self.theta_goal = self.theta_odom+dtheta
        # self.x_goal = self.x_odom+(dx * math.cos(self.theta_odom) - dy*math.sin(self.theta_odom))
        # self.y_goal = self.y_odom+(dx * math.sin(self.theta_odom) + dy*math.cos(self.theta_odom))
        if not (is_stationary) or not (rotation_on):
            # Increasing the P since it's OK while moving
            self.theta_pid = PID(
                p=1.0, i=0.0, d=0.00, max_command=4.0, max_i_contribution=1.0
            )
            self.x_pid = PID(
                p=3.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0
            )
            self.y_pid = PID(
                p=3.0, i=0.00, d=0.0, max_command=0.5, max_i_contribution=0.0
            )

        self.x_pid.set_goal(self.x_goal)
        self.y_pid.set_goal(self.y_goal)
        self.theta_pid.set_goal(self.theta_goal)

        return control_goals_updated

    def main_tick(self, verbose: bool = False):
        """Main function of the HAL node. This function is made to be called often. Handles the main state machine"""
        t = time.time()
        if (not self.scan_is_read) or ((t - self.scan_t0) > self.scan_timeout):
            # If too much time without a LIDAR scan, the speeds are set to 0 for safety.
            self.get_logger().warning(
                "waiting for a LIDAR scan to be read. Discarding all commands..."
            )
            wheel_speeds = self.ik_vel(0.0, 0.0, 0.0)
            self.send_wheel_commands(wheel_speeds)
            time.sleep(0.5)
            return
        if self.first_tick:
            self.first_tick = False
            self.get_logger().info("=> Zuuu HAL up and running! **")

        if self.mode is ZuuuModes.CMD_VEL:
            # If too much time without an order, the speeds are smoothed back to 0 for safety.
            if (self.cmd_vel is not None) and (
                (t - self.cmd_vel_t0) < self.cmd_vel_timeout
            ):
                self.x_vel_goal = self.cmd_vel.linear.x
                self.y_vel_goal = self.cmd_vel.linear.y
                self.theta_vel_goal = self.cmd_vel.angular.z
            else:
                self.x_vel_goal = 0.0
                self.y_vel_goal = 0.0
                self.theta_vel_goal = 0.0
            self.filter_speed_goals()
            # Applying the LIDAR safety
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered,
                self.y_vel_goal_filtered,
                self.theta_vel_goal_filtered,
            )
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is ZuuuModes.BRAKE:
            self.omnibase.back_wheel.set_duty_cycle(0)
            self.omnibase.left_wheel.set_duty_cycle(0)
            self.omnibase.right_wheel.set_duty_cycle(0)
        elif self.mode is ZuuuModes.FREE_WHEEL:
            self.omnibase.back_wheel.set_current(0)
            self.omnibase.left_wheel.set_current(0)
            self.omnibase.right_wheel.set_current(0)
        elif self.mode is ZuuuModes.SPEED:
            if self.speed_service_deadline < time.time():
                if self.speed_service_on:
                    self.get_logger().info(
                        "End of set speed duration, setting speeds to 0"
                    )
                self.speed_service_on = False
                self.x_vel_goal = 0
                self.y_vel_goal = 0
                self.theta_vel_goal = 0
            self.filter_speed_goals()
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered,
                self.y_vel_goal_filtered,
                self.theta_vel_goal_filtered,
            )
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is ZuuuModes.GOTO:
            x_vel, y_vel, theta_vel = 0, 0, 0
            if self.goto_service_on:
                distance = math.sqrt(
                    (self.x_goal - self.x_odom) ** 2 + (self.y_goal - self.y_odom) ** 2
                )
                if (
                    distance < self.xy_tol
                    and abs(angle_diff(self.theta_goal, self.theta_odom))
                    < self.theta_tol
                ):
                    self.goto_service_on = False
                else:
                    x_vel, y_vel, theta_vel = self.position_control()

            x_vel, y_vel, theta_vel = self.lidar_safety.safety_check_speed_command(
                x_vel, y_vel, theta_vel
            )
            wheel_speeds = self.ik_vel(x_vel, y_vel, theta_vel)
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is ZuuuModes.CMD_GOTO:
            if (self.cmd_vel is not None) and (
                (t - self.cmd_vel_t0) < self.cmd_vel_timeout
            ):
                # Normal case, orders where received
                self.x_vel_goal = self.cmd_vel.linear.x
                self.y_vel_goal = self.cmd_vel.linear.y
                self.theta_vel_goal = self.cmd_vel.angular.z
                self.filter_speed_goals()
                control_goals_updated = self.fake_vel_goals_to_goto_goals(
                    self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal
                )

                if control_goals_updated:
                    x_vel, y_vel, theta_vel = self.position_control()
                    # self.get_logger().warning(f"post position control theta_vel {theta_vel:.2f}")

                    x_vel, y_vel, theta_vel = self.limit_vel_commands(
                        x_vel, y_vel, theta_vel
                    )
                    # self.get_logger().warning(f"post limit vel control theta_vel {theta_vel:.2f}")

                    x_vel, y_vel, theta_vel = (
                        self.lidar_safety.safety_check_speed_command(
                            x_vel, y_vel, theta_vel
                        )
                    )
                    # self.get_logger().warning(f"post safety check theta_vel {theta_vel:.2f}")

                    wheel_speeds = self.ik_vel(x_vel, y_vel, theta_vel)
                else:
                    wheel_speeds = self.ik_vel(0.0, 0.0, 0.0)
                self.send_wheel_commands(wheel_speeds)
            else:
                # If too much time without an order, the speeds are smoothed back to 0 for safety.
                self.x_vel_goal = 0.0
                self.y_vel_goal = 0.0
                self.theta_vel_goal = 0.0
                self.filter_speed_goals()
                wheel_speeds = self.ik_vel(
                    self.x_vel_goal_filtered,
                    self.y_vel_goal_filtered,
                    self.theta_vel_goal_filtered,
                )
                self.send_wheel_commands(wheel_speeds)

        elif self.mode is ZuuuModes.EMERGENCY_STOP:
            msg = "Emergency stop requested"
            self.get_logger().warning(msg)
            self.emergency_shutdown()
            raise RuntimeError(msg)

        else:
            self.get_logger().warning(
                "unknown mode '{}', setting it to brake".format(self.mode)
            )
            self.mode = ZuuuModes.BRAKE

        self.old_measure_timestamp = self.measure_timestamp
        self.measure_timestamp = self.get_clock().now()

        # Reading the measurements (this is what takes most of the time, ~9ms)
        self.read_measurements()
        self.update_wheel_speeds()

        if verbose:
            self.print_all_measurements()

        self.publish_wheel_speeds()
        self.tick_odom()

        if verbose:
            self.get_logger().info(
                "x_odom {}, y_odom {}, theta_odom {}".format(
                    self.x_odom, self.y_odom, self.theta_odom
                )
            )

        # Time measurement
        dt = time.time() - t
        if dt == 0:
            f = 0
        else:
            f = 1 / dt
        if verbose:
            self.get_logger().info(
                "zuuu tick potential freq: {:.0f}Hz (dt={:.0f}ms)".format(f, 1000 * dt)
            )


def main(args=None) -> None:
    """Run ZuuuHAL main loop"""
    rclpy.init(args=args)
    try:
        node = ZuuuHAL()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error("Failed to init ZuuuHAL")
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
