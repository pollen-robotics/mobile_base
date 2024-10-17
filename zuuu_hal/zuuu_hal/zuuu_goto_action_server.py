import threading
import time
from queue import Queue
from threading import Event
import math

import numpy as np
import rclpy
from zuuu_interfaces.action import ZuuuGoto
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
)  # ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from zuuu_hal.utils import PID, angle_diff, ZuuuModes, ZuuuControlModes


class ZuuuGotoActionServer(Node):
    def __init__(self, zuuu_hal, shared_callback_group):
        super().__init__(f"mobile_base_goto_action_server")
        self.zuuu_hal = zuuu_hal
        self._goal_queue = Queue()
        self.execution_ongoing = Event()
        
        self.active_goals_lock = threading.Lock()
        self.active_goals = 0

        self._action_server = ActionServer(
            self,
            ZuuuGoto,
            f"mobile_base_goto",
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=shared_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Not sending the feedback every tick
        self.nb_commands_per_feedback = 10
        self.sampling_freq = 100  # Hz
        self.get_logger().info("Zuuu Goto action server init.")
        # create thread for check_queue_and_execute
        self.check_queue_and_execute_thread = threading.Thread(
            target=self.check_queue_and_execute, daemon=True
        )
        self.check_queue_and_execute_thread.start()

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
        
    def increment_active_goals(self):
        with self.active_goals_lock:
            self.active_goals += 1

    def decrement_active_goals(self):
        with self.active_goals_lock:
            self.active_goals -= 1

    def has_active_goals(self):
        with self.active_goals_lock:
            return self.active_goals > 0

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().debug(f"Received goal request: {goal_request.request}")
        # Accepting all goals
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self._goal_queue.put(goal_handle)

    def check_queue_and_execute(self):
        while True:
            goal_handle = self._goal_queue.get()
            self.execution_ongoing.clear()
            self.increment_active_goals()
            goal_handle.execute()
            self.execution_ongoing.wait()
            self.decrement_active_goals()


    def execute_callback(self, goal_handle):
        """Execute a goal."""
        # Link to the documentation:
        # /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/action/server.py
        
        start_time = time.time()
        goto_request = goal_handle.request.request  # zuuu_interfaces/ZuuuGotoRequest
        
        self.get_logger().info(f"Executing goal: {goto_request}")
        
        self.setup_goto(goto_request)
        
        ret = self.goto_time(goal_handle, goto_request)
        if ret == "no_longer_goto_mode":
            # Removing all the goals from the queue for safety
            self.get_logger().warning(f"A mode change happened during the goto execution. Removing all ({self._goal_queue}) goals from the queue.")
            while not self._goal_queue.empty():
                self._goal_queue.get().abort()
        elif not self.keep_control_on_arrival:
            # Chaging the mode to BRAKE when the goal is reached
            self.zuuu_hal.mode = ZuuuModes.BRAKE

        # Populate result message
        result = ZuuuGoto.Result()
        result.result.status = ret
        self.get_logger().info(f"DEBUG PRINT returning result {result}")

        self.execution_ongoing.set()
        return result
    
    def setup_goto(self, goto_request):
        """Setup the goto with the parameters from the request"""
        # Note: the PID values have default values in the goto_request message
        # Setting zuuu_hal mode to GOTO
        self.zuuu_hal.mode = ZuuuModes.GOTO
        self.distance_pid = PID(p=goto_request.distance_p, i=goto_request.distance_i, d=goto_request.distance_d, max_command=goto_request.distance_max_command, max_i_contribution=None)
        self.angle_pid = PID(p=goto_request.angle_p, i=goto_request.angle_i, d=goto_request.angle_d, max_command=goto_request.angle_max_command, max_i_contribution=None)
        self.dist_tol = goto_request.dist_tol
        self.angle_tol = goto_request.angle_tol
        self.keep_control_on_arrival = goto_request.keep_control_on_arrival
        self.distance_pid.set_goal(0.0)
        self.angle_pid.set_goal(0.0)
        # Setting the goal values in zuuu_hal to keep compliance with e.g. DistanceToGoal service resquest
        self.zuuu_hal.x_goal = goto_request.x_goal
        self.zuuu_hal.y_goal = goto_request.y_goal
        self.zuuu_hal.theta_goal = goto_request.theta_goal
    
    def goto_time(self, goal_handle, goto_request):
        """Blocking function that sends commands to the mobile base to reach a goal pose in the odometry frame.
        This function will update the goal_handle state, publish feedback and return when the goal is reached or the timeout is reached
        or the goal is canceled by the client.
        """
        t0 = time.time()
        dt = 1 / self.sampling_freq
        timeout = goto_request.timeout
        commands_sent = 0
        while True:
            # The goal handle status can be changed with succeed(), abort() or canceled()
            t0_loop = time.time()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return "canceled"

            elapsed_time = time.time() - t0
            if elapsed_time > timeout:
                self.get_logger().info(f"goto finished based on timeout")
                goal_handle.abort()
                return "timeout"
            if self.zuuu_hal.mode is ZuuuModes.GOTO :     
                # Performing the control calculations and updating the speed commands in zuuu_hal
                arrived, distance_error, angle_error = self.goto_tick()
                self.zuuu_hal.process_velocity_goals_and_send_wheel_commands()
                if arrived:
                    goal_handle.succeed()
                    return "finished"
            else:
                self.get_logger().info("Zuuu is not in GOTO mode anymore")
                goal_handle.abort()
                return "no_longer_goto_mode"

            # Publishing feedback every nb_commands_per_feedback ticks
            commands_sent += 1
            if commands_sent % self.nb_commands_per_feedback == 0:
                self.generate_and_publish_feedback(goal_handle, distance_error, angle_error)


            # self.rate.sleep()  # Slowly the output freq drops with this... A bug I could not find.
            time.sleep(max(0, dt - (time.time() - t0_loop)))

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f"Received cancel request")

        # Check state and decide
        if goal_handle.is_active or goal_handle.is_executing:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT
        
    def generate_and_publish_feedback(self, goal_handle, distance_error, angle_error):
        feedback_msg = ZuuuGoto.Feedback()
        feedback_msg.feedback.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.feedback.distance_error = distance_error
        feedback_msg.feedback.angle_error = angle_error
        goal_handle.publish_feedback(feedback_msg)

    
    def goto_tick(self):
        """Tick function for the goto mode. Will calculate the robot's speed to reach a goal pose in the odometry frame.
        """
        dx = -self.zuuu_hal.x_goal + self.zuuu_hal.x_odom
        dy = -self.zuuu_hal.y_goal + self.zuuu_hal.y_odom
        distance_error = math.sqrt(dx ** 2 + dy ** 2)
        angle_error = -angle_diff(self.zuuu_hal.theta_goal, self.zuuu_hal.theta_odom)
        arrived = False
        if distance_error < self.dist_tol and abs(angle_error) < self.angle_tol:
            self.zuuu_hal.x_vel_goal, self.zuuu_hal.y_vel_goal, self.zuuu_hal.theta_vel_goal =  0.0, 0.0, 0.0
            self.get_logger().info("Reached the goal position !")
            arrived = True
        else:
            dist_command = self.distance_pid.tick(distance_error)
            angle_command = self.angle_pid.tick(angle_error, is_angle=True)
            # The vector (dx, dy) is the vector from the robot to the goal in the odom frame
            # Transforming that vector from the world-fixed odom frame to the robot-fixed frame
            x_command = dx * math.cos(-self.zuuu_hal.theta_odom) - dy * math.sin(-self.zuuu_hal.theta_odom)
            y_command = dx * math.sin(-self.zuuu_hal.theta_odom) + dy * math.cos(-self.zuuu_hal.theta_odom)
            # Normalizing. The (x_command, y_command) vector is now a unit vector pointing towards the goal in the robot frame
            x_command = x_command / distance_error 
            y_command = y_command / distance_error
            # Scaling the command vector by the PID output
            x_command *= dist_command
            y_command *= dist_command
            
            # No transformations to do with the angle_command as the Z odom axis is coolinear with the Z robot axis
            self.zuuu_hal.x_vel_goal = x_command
            self.zuuu_hal.y_vel_goal = y_command
            self.zuuu_hal.theta_vel_goal = angle_command
            
        return arrived, distance_error, angle_error

