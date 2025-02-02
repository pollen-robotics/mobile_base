"""
Zuuu's Hardware Abstraction Layer main node.
'Hardware' here means the three wheel controllers, the battery and the LIDAR.
The responsibility of the node is to read the sensors, handle common calculations 
(filtering, odometry, inverse kinematics) and expose control interfaces.

See params.yaml for the list of ROS parameters.
"""

import copy
import math
import threading
import time
import traceback
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import rclpy.logging
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from pollen_msgs.msg import MobileBaseState
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import (CallbackGroup,
                                   MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.constants import S_TO_NS
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from reachy_utils.config import ReachyConfig
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from zuuu_interfaces.srv import (DistanceToGoal, GetBatteryVoltage,
                                 GetOdometry, GetZuuuMode, GetZuuuSafety,
                                 ResetOdometry, SetSpeed, SetZuuuMode,
                                 SetZuuuSafety)

from zuuu_hal.kinematics import (dk_vel, ik_vel, pwm_to_wheel_rot_speed,
                                 wheel_rot_speed_to_pwm)
from zuuu_hal.lidar_safety import LidarSafety
from zuuu_hal.mobile_base import MobileBase
from zuuu_hal.utils import PID, ZuuuControlModes, ZuuuModes, angle_diff, sign
from zuuu_hal.zuuu_goto_action_server import ZuuuGotoActionServer


class ZuuuHAL(Node):
    """Zuuu's Hardware Abstraction Layer node."""

    def __init__(self, shared_callback_group: CallbackGroup) -> None:
        """
        Node initialization.
        ROS side: sets up timers, callbacks, topics, services, and parameters.
        Low-level side: connects with the hardware and performs an initial sensor read.
        """
        super().__init__("zuuu_hal")
        self.get_logger().info("Starting zuuu_hal!")

        self.goto_action_server = ZuuuGotoActionServer(self, shared_callback_group)

        # Initialization steps split into helper methods.
        self._init_fake_hardware()
        self._init_version_and_omnibase()
        self.get_logger().info("Reading Zuuu's sensors once...")
        self.read_measurements()
        self._init_parameters()
        self._init_state_variables()
        self._init_pid_controllers()
        self._init_ros_communications()
        self._init_services()
        self._init_transform_broadcaster()
        self._init_misc_timers()

    # -------------------------------------------------------------------------
    # Private initialization helper methods.
    # -------------------------------------------------------------------------

    def _init_fake_hardware(self) -> None:
        self.declare_parameter("fake_hardware", False)
        self.fake_hardware: bool = self.get_parameter("fake_hardware").value

        if self.fake_hardware:
            self.get_logger().info("Running zuuu_hal in fake hardware mode\n")
        else:
            self.get_logger().info("Running zuuu_hal on physical hardware\n")

    def _init_version_and_omnibase(self) -> None:
        # Read version from configuration.
        reachy_config = ReachyConfig()
        self.zuuu_version: str = reachy_config.mobile_base_config["version_hard"]
        self.get_logger().info(f"zuuu version: {self.zuuu_version}")

        try:
            float_model = float(self.zuuu_version)
        except Exception:
            msg = "ZUUU version can't be processed, check that the 'zuuu_version' tag is present " "in the .reachy.yaml file"
            self.get_logger().error(msg)
            self.get_logger().error(traceback.format_exc())
            raise RuntimeError(msg)

        if float_model < 1.0:
            self.omnibase = MobileBase(
                left_wheel_id=24, right_wheel_id=72, back_wheel_id=None, fake_hardware=self.fake_hardware
            )
        elif float_model < 1.2:
            self.omnibase = MobileBase(
                left_wheel_id=24, right_wheel_id=None, back_wheel_id=116, fake_hardware=self.fake_hardware
            )
        else:
            self.omnibase = MobileBase(
                left_wheel_id=None, right_wheel_id=72, back_wheel_id=116, fake_hardware=self.fake_hardware
            )

    def _init_parameters(self) -> None:
        # Declare parameters.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("laser_upper_angle", 2.85),
                ("laser_lower_angle", -2.85),
                ("max_duty_cycle", 0.20),
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
                ("safety_on", True),
            ],
        )
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Ensure required parameters are initialized.
        if self.get_parameter("max_duty_cycle").type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'max_duty_cycle' was not initialized. Check that the param file is provided (using the launch file is the way to go). Shutting down."
            )
            self.destroy_node()
        if self.get_parameter("smoothing_factor").type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                "Parameter 'smoothing_factor' was not initialized. Check that the param file is provided (using the launch file is the way to go). Shutting down."
            )
            self.destroy_node()

        # Parameter initialization.
        self.laser_upper_angle: float = self.get_parameter("laser_upper_angle").get_parameter_value().double_value
        self.laser_lower_angle: float = self.get_parameter("laser_lower_angle").get_parameter_value().double_value
        self.max_duty_cycle: float = self.get_parameter("max_duty_cycle").get_parameter_value().double_value
        self.cmd_vel_timeout: float = self.get_parameter("cmd_vel_timeout").get_parameter_value().double_value
        self.max_full_com_fails: int = self.get_parameter("max_full_com_fails").get_parameter_value().integer_value
        self.main_tick_period: float = self.get_parameter("main_tick_period").get_parameter_value().double_value

        control_mode_param = self.get_parameter("control_mode")
        if control_mode_param.value in [m.name for m in ZuuuControlModes]:
            self.control_mode = ZuuuControlModes[control_mode_param.value]
        else:
            self.get_logger().error(
                f"Parameter 'control_mode' has an unknown value: '{control_mode_param.value}'. Shutting down."
            )
            self.destroy_node()

        self.max_accel_xy: float = self.get_parameter("max_accel_xy").get_parameter_value().double_value
        self.max_accel_theta: float = self.get_parameter("max_accel_theta").get_parameter_value().double_value
        self.max_speed_xy: float = self.get_parameter("max_speed_xy").get_parameter_value().double_value
        self.max_speed_theta: float = self.get_parameter("max_speed_theta").get_parameter_value().double_value
        self.xy_tol: float = self.get_parameter("xy_tol").get_parameter_value().double_value
        self.theta_tol: float = self.get_parameter("theta_tol").get_parameter_value().double_value
        self.smoothing_factor: float = self.get_parameter("smoothing_factor").get_parameter_value().double_value
        self.safety_distance: float = self.get_parameter("safety_distance").get_parameter_value().double_value
        self.critical_distance: float = self.get_parameter("critical_distance").get_parameter_value().double_value
        self.safety_on: bool = self.get_parameter("safety_on").get_parameter_value().bool_value

    def _init_state_variables(self) -> None:
        # Initialize state variables.
        self.cmd_vel: Optional[Twist] = None
        self.x_odom: float = 0.0
        self.y_odom: float = 0.0
        self.theta_odom: float = 0.0
        self.x_odom_gazebo: float = 0.0
        self.y_odom_gazebo: float = 0.0
        self.theta_odom_gazebo: float = 0.0
        self.x_odom_gazebo_old: float = 0.0
        self.y_odom_gazebo_old: float = 0.0
        self.theta_odom_gazebo_old: float = 0.0
        self.theta_zuuu_vs_gazebo: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vtheta: float = 0.0
        self.vx_gazebo: float = 0.0
        self.vy_gazebo: float = 0.0
        self.vtheta_gazebo: float = 0.0
        self.x_vel_goal: float = 0.0
        self.y_vel_goal: float = 0.0
        self.theta_vel_goal: float = 0.0
        self.x_vel_goal_filtered: float = 0.0
        self.y_vel_goal_filtered: float = 0.0
        self.theta_vel_goal_filtered: float = 0.0
        self.x_goal: float = 0.0
        self.y_goal: float = 0.0
        self.theta_goal: float = 0.0
        self.calculated_wheel_speeds: List[float] = [0.0, 0.0, 0.0]
        self.reset_odom: bool = False
        self.battery_voltage: float = 25.0
        self.mode = ZuuuModes.CMD_GOTO  # or CMD_VEL as preferred
        self.speed_service_deadline: float = 0.0
        self.speed_service_on: bool = False
        self.lidar_mandatory: bool = False
        self.scan_is_read: bool = False
        self.scan_timeout: float = 0.5
        self.nb_control_ticks: int = 0
        self.stationary_on: bool = False
        self.already_shutdown: bool = False
        self.nb_full_com_fails: int = 0

        self.lidar_safety = LidarSafety(
            self.safety_distance,
            self.critical_distance,
            robot_collision_radius=0.5,
            speed_reduction_factor=1.0,
            logger=self.get_logger(),
            fake_hardware=self.fake_hardware,
        )
        self.cv_bridge = CvBridge()

    def _init_pid_controllers(self) -> None:
        """
        PID values tuned on Reachy 2 15/10/2024.
        The idea behind this tuning is that the proportional gain (P) is set quite high,
        as it's needed to overcome friction at low speeds.
        However, the maximum command is kept reasonable to limit the cruise speed.

        We also tested a full PID tuning (instead of just P), but the promise of "eliminating the steady-state error"
        does not work well with the small "steps" produced by the holonomic wheel.

        No over-shoot Ziegler Nichols:
        self.theta_pid = PID(p=3.2, i=14.2, d=0.475, max_command=4.0, max_i_contribution=1.0)
        """
        self.distance_pid_cmd_goto = PID(p=5.0, i=0.0, d=0.0, max_command=0.4, max_i_contribution=0.2)
        self.angle_pid_cmd_goto = PID(p=5.0, i=0.0, d=0.0, max_command=1.0, max_i_contribution=0.5)
        # TODO test IRL if this is a good idea. Not using it for now
        # self.slow_distance_pid_cmd_goto = PID(p=1.0, i=0.0, d=0.0, max_command=0.4, max_i_contribution=0.2)
        # self.slow_angle_pid_cmd_goto = PID(p=1.0, i=0.0, d=0.0, max_command=1.0, max_i_contribution=0.5)

        self.distance_pid = PID(p=5.0, i=0.0, d=0.0, max_command=0.4, max_i_contribution=0.2)
        self.angle_pid = PID(p=5.0, i=0.0, d=0.0, max_command=1.0, max_i_contribution=0.5)

        self.max_wheel_speed = pwm_to_wheel_rot_speed(self.max_duty_cycle)
        self.get_logger().info(
            f"The maximum PWM value is {self.max_duty_cycle * 100}% => maximum wheel speed is set to {self.max_wheel_speed:.2f} rad/s"
        )

    def _init_ros_communications(self) -> None:
        # Subscriptions.
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_filter_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if self.fake_hardware:
            # In Gazebo mode subscribe to the odom topic published by the gazebo plugin.
            self.odom_sub = self.create_subscription(
                Odometry,
                "odom",
                self.gazebo_odom_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        self.scan_pub = self.create_publisher(LaserScan, "scan_filterd", 10)
        self.lidar_image_pub = self.create_publisher(Image, "lidar_image", 1)

        self.pub_back_wheel_rpm = self.create_publisher(Float32, "back_wheel_rpm", 2)
        self.pub_left_wheel_rpm = self.create_publisher(Float32, "left_wheel_rpm", 2)
        self.pub_right_wheel_rpm = self.create_publisher(Float32, "right_wheel_rpm", 2)
        self.pub_mobile_base_state = self.create_publisher(MobileBaseState, "mobile_base_state", 2)
        self.pub_odom = self.create_publisher(Odometry, "odom_zuuu", 2)
        self.pub_fake_vel = self.create_publisher(Twist, "cmd_vel_gazebo", 10)

    def _init_services(self) -> None:
        # Services.
        self.mode_service = self.create_service(SetZuuuMode, "SetZuuuMode", self.handle_zuuu_mode)
        self.get_mode_service = self.create_service(GetZuuuMode, "GetZuuuMode", self.handle_get_zuuu_mode)
        self.reset_odometry_service = self.create_service(ResetOdometry, "ResetOdometry", self.handle_reset_odometry)
        self.get_odometry_service = self.create_service(GetOdometry, "GetOdometry", self.handle_get_odometry)
        self.set_speed_service = self.create_service(SetSpeed, "SetSpeed", self.handle_set_speed)
        self.distance_to_goal = self.create_service(DistanceToGoal, "DistanceToGoal", self.handle_distance_to_goal)
        self.get_battery_voltage_service = self.create_service(
            GetBatteryVoltage, "GetBatteryVoltage", self.handle_get_battery_voltage
        )
        self.set_safety_service = self.create_service(SetZuuuSafety, "SetZuuuSafety", self.handle_zuuu_set_safety)
        self.get_safety_service = self.create_service(GetZuuuSafety, "GetZuuuSafety", self.handle_zuuu_get_safety)

    def _init_transform_broadcaster(self) -> None:
        # Initialize transform broadcaster.
        self.br = TransformBroadcaster(self)
        self.old_measure_timestamp = self.get_clock().now()
        self.measure_timestamp = self.get_clock().now()

    def _init_misc_timers(self) -> None:
        # Miscellaneous timers and measurements.
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

        # Timers.
        self.create_timer(self.main_tick_period, self.main_tick)
        self.measurements_t = time.time()
        self.check_battery()  # Check battery once at startup.
        self.create_timer(self.omnibase.battery_check_period, self.check_battery)

    def parameters_callback(self, params) -> SetParametersResult:
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
                elif param.name == "max_duty_cycle":
                    if param.value >= 0.0 and param.value <= 1.0:
                        self.max_duty_cycle = param.value
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
            elif param.type_ is Parameter.Type.BOOL:
                if param.name == "safety_on":
                    self.safety_on = param.value
                    success = True

        return SetParametersResult(successful=success)

    def handle_zuuu_mode(self, request: SetZuuuMode.Request, response: SetZuuuMode.Response) -> SetZuuuMode.Response:
        """Handle SetZuuuMode service request"""
        self.get_logger().info(f"Requested mode change to :'{request.mode}'")
        response.success = False

        if request.mode in [m.name for m in ZuuuModes]:
            if request.mode == ZuuuModes.SPEED.name:
                self.get_logger().info(f"'{request.mode}' should not be changed by hand, use the SetSpeed service instead")
            elif request.mode == ZuuuModes.GOTO.name:
                self.get_logger().info(
                    f"'{request.mode}' should not be changed by hand, use the ZuuuGotoActionServer action server instead"
                )
            else:
                # Changing the mode is a way to prematurely end an on going task requested through a service
                self.stop_ongoing_services()
                self.mode = ZuuuModes[request.mode]
                response.success = True
                self.get_logger().info("OK")

        return response

    def handle_get_zuuu_mode(self, request: GetZuuuMode.Request, response: GetZuuuMode.Response) -> GetZuuuMode.Response:
        """Handle GetZuuuMode service request"""
        response.mode = self.mode.name
        return response

    def handle_reset_odometry(self, request: ResetOdometry.Request, response: ResetOdometry.Response) -> ResetOdometry.Response:
        """Handle ResetOdometry service request"""
        # Resetting asynchronously to prevent race conditions.
        self.reset_odom = True
        self.get_logger().info("Requested to reset the odometry frame")
        response.success = True
        return response

    def handle_get_odometry(self, request: GetOdometry.Request, response: GetOdometry.Response) -> GetOdometry.Response:
        response.x = self.x_odom
        response.y = self.y_odom
        response.theta = self.theta_odom
        response.vx = self.vx
        response.vy = self.vy
        response.vtheta = self.vtheta
        return response

    def handle_set_speed(self, request: SetSpeed.Request, response: SetSpeed.Response) -> SetSpeed.Response:
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

    def handle_distance_to_goal(
        self, request: DistanceToGoal.Request, response: DistanceToGoal.Response
    ) -> DistanceToGoal.Response:
        """Handle DistanceToGoal service resquest"""
        response.delta_x = self.x_goal - self.x_odom
        response.delta_y = self.y_goal - self.y_odom
        response.delta_theta = angle_diff(self.theta_goal, self.theta_odom)
        response.distance = math.sqrt((self.x_goal - self.x_odom) ** 2 + (self.y_goal - self.y_odom) ** 2)
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

    def publish_mobile_base_state(self) -> None:
        """Publishes the safety status in the `mobile_base_safety_status` topic"""
        msg = MobileBaseState()
        string_status = self.lidar_safety.obstacle_detection_status
        t = time.time()
        if (not self.scan_is_read) or ((t - self.scan_t0) > self.scan_timeout):
            # Too much time without a LIDAR scan
            string_status = "error"

        # string to float conversion using the mobile_base_lidar.proto file
        # [0: detection error, 1: no obstacle, 2: obstacle detected slowing down, 3: obstacle detected stopping]
        status = 0.0  # no object detected (DETECTION_ERROR)
        if string_status == "green":
            status = 1.0  # no object detected (NO_OBJECT_DETECTED)
        elif string_status == "orange":
            status = 2.0  # object detected but not critical (OBJECT_DETECTED_SLOWDOWN)
        elif string_status == "red":
            status = 3.0  # object detected and critical (OBJECT_DETECTED_STOP)

        msg.safety_on.data = self.safety_on
        # construct the Float32MultiArray message
        msg.mobile_base_safety_status.data = [
            self.lidar_safety.safety_distance,
            self.lidar_safety.critical_distance,
            status,
        ]

        msg.battery_voltage = Float32(data=self.battery_voltage)

        msg.zuuu_mode = self.mode.name
        msg.control_mode = self.control_mode.name

        self.pub_mobile_base_state.publish(msg)

    def check_battery(self, verbose: bool = False) -> None:
        """Checks that the battery readings are not too old and forces a read if need be.
        Checks that the battery voltages are safe and warns or stops the HAL accordingly.
        """
        if self.fake_hardware:
            return
        t = time.time()
        if verbose:
            self.print_all_measurements()
        if (t - self.measurements_t) > (self.omnibase.battery_check_period + 1):
            self.get_logger().warning("Zuuu's measurements are not made often enough. Reading now.")
            self.read_measurements()
        warn_voltage = self.omnibase.battery_nb_cells * self.omnibase.battery_cell_warn_voltage
        min_voltage = self.omnibase.battery_nb_cells * self.omnibase.battery_cell_min_voltage
        voltage = self.battery_voltage

        if min_voltage < voltage < warn_voltage:
            self.get_logger().warning(
                f"Battery voltage LOW ({voltage}V). Consider recharging. Warning threshold: {warn_voltage:.1f}V, stop threshold: {min_voltage:.1f}V"
            )
        elif voltage < min_voltage:
            msg = f"Battery voltage critically LOW ({voltage}V). Emergency shutdown! Warning threshold: {min_voltage:.1f}V, "
            self.emergency_shutdown(msg)

        else:
            self.get_logger().warning(f"Battery voltage OK ({voltage}V)")

    def emergency_shutdown(self, msg: str) -> None:
        """
        Sets the PWM of the three wheel motors to 0V (i.e., brakes the system) and raises a RuntimeError.

        """
        self.get_logger().warn(f"Emergency shutdown initiated in zuuu_hal: {msg}")

        if self.already_shutdown:
            return

        try:
            if not self.fake_hardware:
                self.omnibase.back_wheel.set_duty_cycle(0)
                self.omnibase.left_wheel.set_duty_cycle(0)
                self.omnibase.right_wheel.set_duty_cycle(0)
            else:
                self.publish_fake_robot_speed(0, 0, 0)
        except Exception as hardware_error:
            self.get_logger().error(f"Error during hardware shutdown: {hardware_error}")
        finally:
            self.already_shutdown = True

        time.sleep(0.1)

        raise RuntimeError(msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handles the callback on the /cmd_vel topic"""
        self.cmd_vel = msg
        self.cmd_vel_t0 = time.time()

    def scan_filter_callback(self, msg: LaserScan) -> None:
        """Callback method on the /scan topic. Handles the LIDAR filtering and safety calculations."""
        self.scan_is_read = True
        self.scan_t0 = time.time()
        # LIDAR angle filter managemnt
        # https://github.com/ros-perception/laser_filters was broken in ROS2 last time I checked
        # => Doing it manually
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
        angle_min_offset = 0.0
        if self.fake_hardware:
            angle_min_offset = -math.pi
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + angle_min_offset + i * msg.angle_increment
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

    def gazebo_odom_callback(self, msg: Odometry) -> None:
        """Callback method on the /odom topic used in Gazebo mode"""
        self.vx_gazebo = msg.twist.twist.linear.x
        self.vy_gazebo = msg.twist.twist.linear.y
        self.vtheta_gazebo = msg.twist.twist.angular.z

        # reading the odometry from the gazebo plugin
        self.x_odom_gazebo = msg.pose.pose.position.x
        self.y_odom_gazebo = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta_odom_gazebo = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def filter_speed_goals(self, x_vel, y_vel, theta_vel):
        """Applies a smoothing filter"""
        self.x_vel_goal_filtered = (x_vel + self.smoothing_factor * self.x_vel_goal_filtered) / (1 + self.smoothing_factor)
        self.y_vel_goal_filtered = (y_vel + self.smoothing_factor * self.y_vel_goal_filtered) / (1 + self.smoothing_factor)
        self.theta_vel_goal_filtered = (theta_vel + self.smoothing_factor * self.theta_vel_goal_filtered) / (
            1 + self.smoothing_factor
        )
        return self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered

    def format_measurements(self, measurements) -> str:
        """Text formatting for the low level controller measurements"""
        if measurements is None:
            return "None"
        to_print = ""
        to_print += f"temp_fet:{measurements.temp_fet}\n"
        to_print += f"temp_motor:{measurements.temp_motor}\n"
        to_print += f"avg_motor_current:{measurements.avg_motor_current}\n"
        to_print += f"avg_input_current:{measurements.avg_input_current}\n"
        to_print += f"avg_id:{measurements.avg_id}\n"
        to_print += f"avg_iq:{measurements.avg_iq}\n"
        to_print += f"duty_cycle_now:{measurements.duty_cycle_now}\n"
        to_print += f"rpm:{measurements.rpm}\n"
        to_print += f"v_in:{measurements.v_in}\n"
        to_print += f"amp_hours:{measurements.amp_hours}\n"
        to_print += f"amp_hours_charged:{measurements.amp_hours_charged}\n"
        to_print += f"watt_hours:{measurements.watt_hours}\n"
        to_print += f"watt_hours_charged:{measurements.watt_hours_charged}\n"
        to_print += f"tachometer:{measurements.tachometer}\n"
        to_print += f"tachometer_abs:{measurements.tachometer_abs}\n"
        to_print += f"mc_fault_code:{measurements.mc_fault_code}\n"
        to_print += f"pid_pos_now:{measurements.pid_pos_now}\n"
        to_print += f"app_controller_id:{measurements.app_controller_id}\n"
        to_print += f"time_ms:{measurements.time_ms}\n"
        return to_print

    def print_all_measurements(self) -> None:
        """Prints the low level measurements from the 3 wheel controllers"""
        to_print = "\n*** back_wheel measurements:\n"
        to_print += self.format_measurements(self.omnibase.back_wheel_measurements)
        to_print += "\n\n*** left_wheel:\n"
        to_print += self.format_measurements(self.omnibase.left_wheel_measurements)
        to_print += "\n\n*** right_wheel:\n"
        to_print += self.format_measurements(self.omnibase.right_wheel_measurements)
        to_print += f"\n\n Fails ('Nones') left:{self.omnibase.left_wheel_nones}, right:{self.omnibase.right_wheel_nones}, back:{self.omnibase.back_wheel_nones}"
        to_print += f"\n\n AVG RPM left:{self.omnibase.left_wheel_avg_rpm / self.omnibase.half_poles:.2f}, right:{self.omnibase.right_wheel_avg_rpm / self.omnibase.half_poles:.2f}, back:{self.omnibase.back_wheel_avg_rpm / self.omnibase.half_poles:.2f}"
        self.get_logger().info(f"{to_print}")

    def publish_wheel_speeds(self) -> None:
        """Publishes the most recent measures of rotational speed for each of the 3 wheels on 3 separate topics."""
        # If the measurements are None, not publishing
        if self.omnibase.back_wheel_measurements is not None:
            rpm_back = Float32()
            if self.fake_hardware:
                rpm_back.data = float(self.calculated_wheel_speeds[0])
            else:
                rpm_back.data = float(self.omnibase.back_wheel_measurements.rpm)
            self.pub_back_wheel_rpm.publish(rpm_back)

        if self.omnibase.right_wheel_measurements is not None:
            rpm_right = Float32()
            if self.fake_hardware:
                rpm_right.data = float(self.calculated_wheel_speeds[1])
            else:
                rpm_right.data = float(self.omnibase.right_wheel_measurements.rpm)
            self.pub_right_wheel_rpm.publish(rpm_right)

        if self.omnibase.left_wheel_measurements is not None:
            rpm_left = Float32()
            if self.fake_hardware:
                rpm_left.data = float(self.calculated_wheel_speeds[2])
            else:
                rpm_left.data = float(self.omnibase.left_wheel_measurements.rpm)
            self.pub_left_wheel_rpm.publish(rpm_left)

    def update_wheel_speeds(self) -> None:
        """Uses the latest mesure of wheel rotational speed to update the smoothed internal estimation of the wheel
        rotational speed
        """
        # Keeping a local value of the wheel speeds to handle None measurements (we'll use the last valid measure)
        if self.omnibase.back_wheel_measurements is not None:
            value = float(self.omnibase.back_wheel_measurements.rpm)
            self.omnibase.back_wheel_rpm = value
            self.omnibase.back_wheel_rpm_deque.appendleft(value)
            self.omnibase.back_wheel_avg_rpm = self.omnibase.deque_to_avg(self.omnibase.back_wheel_rpm_deque)
        else:
            self.omnibase.back_wheel_nones += 1

        if self.omnibase.left_wheel_measurements is not None:
            value = float(self.omnibase.left_wheel_measurements.rpm)
            self.omnibase.left_wheel_rpm = value
            self.omnibase.left_wheel_rpm_deque.appendleft(value)
            self.omnibase.left_wheel_avg_rpm = self.omnibase.deque_to_avg(self.omnibase.left_wheel_rpm_deque)
        else:
            self.omnibase.left_wheel_nones += 1

        if self.omnibase.right_wheel_measurements is not None:
            value = float(self.omnibase.right_wheel_measurements.rpm)
            self.omnibase.right_wheel_rpm = value
            self.omnibase.right_wheel_rpm_deque.appendleft(value)
            self.omnibase.right_wheel_avg_rpm = self.omnibase.deque_to_avg(self.omnibase.right_wheel_rpm_deque)
        else:
            self.omnibase.right_wheel_nones += 1

    def publish_odometry_and_tf(self) -> None:
        """Publishes the current odometry position (Odometry type published on the /odom topic) and also
        published the TransformStamped between the frame base_footprint and odom
        """
        # Odom
        odom = Odometry()
        odom.header.frame_id = "odom_zuuu"
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

        # Tune these numbers if needed
        odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
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

    def publish_fake_robot_speed(self, x_vel, y_vel, theta_vel) -> None:
        """Publishes the current robot speed (Twist type)"""
        twist = Twist()
        twist.linear.x = float(x_vel)
        twist.linear.y = float(y_vel)
        twist.angular.z = float(theta_vel)
        # self.get_logger().info(f"Publishing fake robot speed: x={x_vel:.2f}m/s, y={y_vel:.2f}m/s, theta={theta_vel:.2f}rad/s")

        self.pub_fake_vel.publish(twist)

    def odom_tick(self) -> None:
        """Updates the odometry values based on the small displacement measured since the last tick,
        then publishes the results with publish_odometry_and_tf()
        """
        self.old_measure_timestamp = self.measure_timestamp
        self.measure_timestamp = self.get_clock().now()
        dt_duration = self.measure_timestamp - self.old_measure_timestamp
        dt_seconds = dt_duration.nanoseconds / S_TO_NS
        if dt_seconds == 0:
            return
        if not self.fake_hardware:
            # Note: VESC speed values are, as is normal, very noisy at low speeds.
            # We currently have no control on how the speeds are calculated.
            # -> By reading the encoder ticks directly and making the calculations here we could maybe make this a tad better?

            # Local speeds in egocentric frame deduced from the wheel speeds
            # "rpm" are actually erpm and need to be divided by half the amount of magnetic poles to get the actual rpm.
            pole_factor = self.omnibase.half_poles
            x_vel, y_vel, theta_vel = dk_vel(
                self.omnibase.left_wheel_rpm * pole_factor,
                self.omnibase.right_wheel_rpm * pole_factor,
                self.omnibase.back_wheel_rpm * pole_factor,
                self.omnibase,
            )
            # Applying the small displacement in the world-fixed odom frame (simple 2D rotation)
            dx = (x_vel * math.cos(self.theta_odom) - y_vel * math.sin(self.theta_odom)) * dt_seconds
            dy = (x_vel * math.sin(self.theta_odom) + y_vel * math.cos(self.theta_odom)) * dt_seconds
            dtheta = theta_vel * dt_seconds
            self.x_odom += dx
            self.y_odom += dy
            self.theta_odom += dtheta
            # These speeds are expected in the ego-centric frame
            self.vx = x_vel
            self.vy = y_vel
            self.vtheta = theta_vel
        else:
            # Note: Using self.vx_gazebo and calculating the odometry from it as in the real case is a way of creating a noisy odometry
            # which is useful in certain situations. The version below uses the gazebo odometry directly which is much more precise.
            self.vx = self.vx_gazebo
            self.vy = self.vy_gazebo
            self.vtheta = self.vtheta_gazebo

            # This is the small displacement in the world-fixed Gazebo odom frame (that never resets)
            dx_gazebo = self.x_odom_gazebo - self.x_odom_gazebo_old
            dy_gazebo = self.y_odom_gazebo - self.y_odom_gazebo_old
            # Since our odom frame can reset, we must apply a 2D rotation to the displacement to get it in our odom frame
            dx = dx_gazebo * math.cos(self.theta_zuuu_vs_gazebo) - dy_gazebo * math.sin(self.theta_zuuu_vs_gazebo)
            dy = dx_gazebo * math.sin(self.theta_zuuu_vs_gazebo) + dy_gazebo * math.cos(self.theta_zuuu_vs_gazebo)
            dtheta = angle_diff(self.theta_odom_gazebo, self.theta_odom_gazebo_old)

            self.x_odom += dx
            self.y_odom += dy
            self.theta_odom += dtheta

            # self.x_odom = self.x_odom_gazebo
            # self.y_odom = self.y_odom_gazebo
            # self.theta_odom = self.theta_odom_gazebo

            self.x_odom_gazebo_old = self.x_odom_gazebo
            self.y_odom_gazebo_old = self.y_odom_gazebo
            self.theta_odom_gazebo_old = self.theta_odom_gazebo

        if self.reset_odom:
            # Resetting asynchronously to prevent race conditions.
            # dx, dy and dteta remain correct even on the reset tick
            self.reset_odom = False
            self.reset_odom_now()

        self.publish_odometry_and_tf()
        # self.get_logger().info(f"XXX Odom: x={self.x_odom:.2f}m, y={self.y_odom:.2f}m, theta={self.theta_odom:.2f}rad")

    def reset_odom_now(self):
        if self.mode is ZuuuModes.GOTO and self.goto_action_server.has_active_goals():
            # Resetting the odometry while a GoTo is ON might be dangerous. Stopping it to make sure:
            self.get_logger().warning("Resetting the odometry while a GoTo is ON. Setting the mode to BRAKE for safety.")
            self.mode = ZuuuModes.BRAKE

        if self.fake_hardware:
            # TODO this is not enough, there is an accumulation of error when resetting the odometry in Gazebo mode
            self.theta_zuuu_vs_gazebo -= self.theta_odom

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

    def limit_duty_cycles(self, duty_cycles: List[float]) -> List[float]:
        """Limits the duty cycles to stay in +-max_duty_cycle"""
        for i in range(len(duty_cycles)):
            if duty_cycles[i] < 0:
                temp = max(-self.max_duty_cycle, duty_cycles[i])
                if temp != duty_cycles[i]:
                    self.get_logger().warning("Duty cycle is LIMITED")
                duty_cycles[i] = temp
            else:
                temp = min(self.max_duty_cycle, duty_cycles[i])
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
        if self.fake_hardware:
            return
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
                self.emergency_shutdown(msg)
            return
        # Read success
        self.nb_full_com_fails = 0
        self.measurements_t = time.time()

    def send_wheel_commands(self, wheel_speeds: List[float]) -> None:
        """Sends either a PWM command or a speed command to the wheel controllers, based on the current control mode"""
        if self.control_mode is ZuuuControlModes.OPEN_LOOP:
            duty_cycles = [wheel_rot_speed_to_pwm(wheel_speed) for wheel_speed in wheel_speeds]
            duty_cycles = self.limit_duty_cycles(duty_cycles)
            self.omnibase.back_wheel.set_duty_cycle(duty_cycles[0])
            self.omnibase.left_wheel.set_duty_cycle(duty_cycles[2])
            self.omnibase.right_wheel.set_duty_cycle(duty_cycles[1])
        elif self.control_mode is ZuuuControlModes.PID:
            # rad/s to rpm to erpm
            wheel_speeds = self.limit_wheel_speeds(wheel_speeds)
            self.omnibase.back_wheel.set_rpm(int(self.omnibase.half_poles * wheel_speeds[0] * 30 / math.pi))
            self.omnibase.left_wheel.set_rpm(int(self.omnibase.half_poles * wheel_speeds[2] * 30 / math.pi))
            self.omnibase.right_wheel.set_rpm(int(self.omnibase.half_poles * wheel_speeds[1] * 30 / math.pi))
        else:
            self.get_logger().warning(f"unknown control mode '{self.control_mode}'")

    def stop_ongoing_services(self) -> None:
        """Stops the SetSpeed service, if it was running"""
        self.speed_service_on = False

    def handle_joy_discretization(self, dx, dy, dtheta, almost_zero=0.001, nb_directions=8):
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
                self.emergency_shutdown(msg)

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
        dx = x_vel_goal
        dy = y_vel_goal
        dtheta = theta_vel_goal
        # # V0 no smart correction, everything is open
        # self.theta_goal = self.theta_odom+dtheta
        # self.x_goal = self.x_odom+(dx * math.cos(self.theta_odom) - dy*math.sin(self.theta_odom))
        # self.y_goal = self.y_odom+(dx * math.sin(self.theta_odom) + dy*math.cos(self.theta_odom))
        # return
        almost_zero = 0.001
        # doing goal_theta when not rotating and current theta when rotating
        (
            joy_angle,
            intensity,
            rotation_on,
            direction_changed,
            rotation_changed,
            is_stationary,
        ) = self.handle_joy_discretization(dx, dy, dtheta, almost_zero=almost_zero, nb_directions=8)

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
                # TODO removing this mecanism for now, test IRL and see if it's needed
                pass
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

    def cmd_vel_tick(self):
        t = time.time()
        # If too much time without an order, the speeds are smoothed back to 0 for safety.
        if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout):
            self.x_vel_goal = self.cmd_vel.linear.x
            self.y_vel_goal = self.cmd_vel.linear.y
            self.theta_vel_goal = self.cmd_vel.angular.z
        else:
            self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal = 0.0, 0.0, 0.0

    def speed_mode_tick(self) -> None:
        """Tick function for the speed mode. Will only set the speeds to 0 if the duration is over."""
        if self.speed_service_deadline < time.time():
            if self.speed_service_on:
                self.get_logger().info("End of set speed duration, setting speeds to 0")
            self.speed_service_on = False
            self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal = 0.0, 0.0, 0.0

    def goto_tick(self, shortest_angle=False, distance_pid=None, angle_pid=None):
        """Tick function for the goto mode. Will calculate the robot's speed to reach a goal pose in the odometry frame.
        This function uses a PID controlle for translation and a PID controller for rotation.
        """
        if distance_pid is None:
            distance_pid = self.distance_pid
        else:
            distance_pid = distance_pid

        if angle_pid is None:
            angle_pid = self.angle_pid
        else:
            angle_pid = angle_pid

        dx = self.x_odom - self.x_goal
        dy = self.y_odom - self.y_goal
        distance_error = math.sqrt(dx**2 + dy**2)
        angle_error = self.theta_odom - self.theta_goal

        dist_command = distance_pid.tick(distance_error)
        if not shortest_angle:
            # This version gives full control to the user
            angle_command = angle_pid.tick(angle_error)
        else:
            # With this version the robot will rotate towards the goal with the shortest path
            angle_command = angle_pid.tick(angle_error, is_angle=True)

        if distance_error == 0:
            x_command = 0
            y_command = 0
        else:
            # The vector (dx, dy) is the vector from the robot to the goal in the odom frame
            # Transforming that vector from the world-fixed odom frame to the robot-fixed frame
            x_command = dx * math.cos(-self.theta_odom) - dy * math.sin(-self.theta_odom)
            y_command = dx * math.sin(-self.theta_odom) + dy * math.cos(-self.theta_odom)

            # Normalizing. The (x_command, y_command) vector is now a unit vector pointing towards the goal in the robot frame
            x_command = x_command / distance_error
            y_command = y_command / distance_error
            # Scaling the command vector by the PID output
            x_command *= dist_command
            y_command *= dist_command

        # No transformations to do with the angle_command as the Z odom axis is coolinear with the Z robot axis
        self.x_vel_goal = x_command
        self.y_vel_goal = y_command
        self.theta_vel_goal = angle_command

    def cmd_goto_tick(self):
        t = time.time()
        # If too much time without an order, the speeds are smoothed back to 0 for safety.
        if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout):
            # Normal case, orders where received
            self.x_vel_goal = self.cmd_vel.linear.x
            self.y_vel_goal = self.cmd_vel.linear.y
            self.theta_vel_goal = self.cmd_vel.angular.z
            self.fake_vel_goals_to_goto_goals(self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal)
            self.goto_tick(shortest_angle=False, distance_pid=self.distance_pid_cmd_goto, angle_pid=self.angle_pid_cmd_goto)
        else:
            self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal = 0.0, 0.0, 0.0

    def handle_stop_modes(self, mode):
        self.calculated_wheel_speeds = [0.0, 0.0, 0.0]
        # To avoid smoothing shenanigans if we go back to a speed mode
        self.x_vel_goal_filtered = 0.0
        self.y_vel_goal_filtered = 0.0
        self.theta_vel_goal_filtered = 0.0
        if mode is ZuuuModes.BRAKE:
            if not self.fake_hardware:
                self.omnibase.back_wheel.set_duty_cycle(0)
                self.omnibase.left_wheel.set_duty_cycle(0)
                self.omnibase.right_wheel.set_duty_cycle(0)
            else:
                self.publish_fake_robot_speed(0, 0, 0)

        elif mode is ZuuuModes.FREE_WHEEL:
            if not self.fake_hardware:
                self.omnibase.back_wheel.set_current(0)
                self.omnibase.left_wheel.set_current(0)
                self.omnibase.right_wheel.set_current(0)
            else:
                self.publish_fake_robot_speed(0, 0, 0)
        else:
            msg = f"Unknown mode requested: '{mode}' => calling emergency_shutdown"
            self.emergency_shutdown(msg)

    def control_tick(self):
        if self.mode in ZuuuModes.speed_modes():
            # Speed modes will perform different types of calculation, but will always output speeds in the robot's frame
            if self.mode is ZuuuModes.CMD_VEL:
                self.cmd_vel_tick()
            elif self.mode is ZuuuModes.SPEED:
                self.speed_mode_tick()
            elif self.mode is ZuuuModes.GOTO:
                self.goto_tick()
            elif self.mode is ZuuuModes.CMD_GOTO:
                self.cmd_goto_tick()
            # Here, the following values have been calculated: self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal
            self.process_velocity_goals_and_send_wheel_commands()
        else:
            # Stop modes directly send stopping commands to the wheels
            self.handle_stop_modes(self.mode)

    def process_velocity_goals_and_send_wheel_commands(self):
        """Processes the robot speed goals and sends the calculated wheel speeds to the wheel controllers.
        The robot speed goals are filtered, safety checked and then transformed into wheel speeds using the IK.
        This function is publisdes differently depending on the mode of the robot (fake or real hardware).
        """
        x_vel, y_vel, theta_vel = self.filter_speed_goals(self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal)
        x_vel, y_vel, theta_vel = self.lidar_safety.safety_check_speed_command(x_vel, y_vel, theta_vel)
        x_vel, y_vel, theta_vel = self.limit_vel_commands(x_vel, y_vel, theta_vel)

        # IK calculations. From Robot's speed to wheels' speeds
        self.calculated_wheel_speeds = ik_vel(x_vel, y_vel, theta_vel, self.omnibase)
        if self.fake_hardware:
            # In fake or Gazebo mode, the robot's speed is published directly and the mouvement is simulated
            self.publish_fake_robot_speed(x_vel, y_vel, theta_vel)
        else:
            # Sending the commands to the physical wheels
            self.send_wheel_commands(self.calculated_wheel_speeds)

    def measurements_tick(self, verbose: bool = False):
        if not self.fake_hardware:
            # Reading the measurements (this is what takes most of the time, ~9ms).
            self.read_measurements()
            self.update_wheel_speeds()
            self.publish_wheel_speeds()
            if verbose:
                self.print_all_measurements()

        self.publish_wheel_speeds()
        self.publish_mobile_base_state()

    def check_for_lidar_scan(self, t):
        if (not self.scan_is_read) or ((t - self.scan_t0) > self.scan_timeout):
            # If too much time without a LIDAR scan, the speeds are set to 0 for safety.
            self.get_logger().warning("waiting for a LIDAR scan to be read. Discarding all commands...")
            wheel_speeds = self.ik_vel(0.0, 0.0, 0.0, self.omnibase)
            self.send_wheel_commands(wheel_speeds)
            time.sleep(0.5)
            return False
        return True

    def print_loop_freq(self, t):
        dt = time.time() - t
        if dt == 0:
            f = 0.0
        else:
            f = 1.0 / dt
        self.get_logger().info("zuuu tick potential freq: {f:.0f}Hz (dt={1000 * dt:.0f}ms)")

    def main_tick(self, verbose: bool = False):
        """Main function of the HAL node. This function is made to be called often. Handles the main state machine"""
        t = time.time()
        if self.lidar_mandatory and not self.check_for_lidar_scan(t):
            return

        if self.first_tick:
            self.first_tick = False
            self.get_logger().info("=> Zuuu HAL up and running! **")

        self.control_tick()

        self.measurements_tick(verbose)

        self.odom_tick()

        if verbose:
            self.print_loop_freq(t)


def main(args=None) -> None:
    """Run ZuuuHAL main loop"""
    try:
        rclpy.init(args=args)

        callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # ReentrantCallbackGroup() brings bad memories, avoiding it if possible

        zuuu_hal = ZuuuHAL(callback_group)

        mult_executor = MultiThreadedExecutor()
        mult_executor.add_node(zuuu_hal)
        mult_executor.add_node(zuuu_hal.goto_action_server)
        executor_thread = threading.Thread(target=mult_executor.spin, daemon=True)
        executor_thread.start()
        rate = zuuu_hal.create_rate(2.0)

        while rclpy.ok():
            rclpy.logging._root_logger.info("tick")
            rate.sleep()
    except KeyboardInterrupt:
        # rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error("KeyboardInterrupt in zuuu_hal")
    finally:
        zuuu_hal.emergency_shutdown("Default zuuu_hal shutdown")
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()
