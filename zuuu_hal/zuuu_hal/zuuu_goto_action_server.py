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
        self.nb_remaining_deletions = 0
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

            
    def cancel_all_future_goals(self):
        self.get_logger().info(f"cancelling all future goals")
        
        while not self._goal_queue.empty():
            server_goal_handle = self._goal_queue.get()
            server_goal_handle.abort()
            # server_goal_handle.destroy()

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        # Link to the documentation:
        # /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/action/server.py
        try :
            
            if self.nb_remaining_deletions == 0:
                # Normal execution
                goto_request = goal_handle.request.request  # zuuu_interfaces/ZuuuGotoRequest
                
                self.get_logger().info(f"Executing goal: {goto_request}")
                
                self.setup_goto(goto_request)
                
                ret = self.goto_time(goal_handle, goto_request)
                if ret == "no_longer_goto_mode":
                    # Removing all the goals from the queue for safety
                    self.nb_remaining_deletions = self._goal_queue.qsize()
                    self.get_logger().warning(f"A mode change happened during the goto execution. Removing all ({self.nb_remaining_deletions}) goals from the queue.")
                    # Note: a method to cancel all goals would have been nice, but accepted goals can't be canceled nor aborted before being executed:
                    # https://design.ros2.org/articles/actions.html
                elif ret == "canceled":
                    q_size = self._goal_queue.qsize()
                    if q_size == 0:
                        self.get_logger().info(f"Goal canceled and no other goals in the queue => Brake mode")
                        self.zuuu_hal.mode = ZuuuModes.BRAKE     
                elif (not self.keep_control_on_arrival) and (self._goal_queue.qsize() == 0):
                    # Changing the mode to BRAKE when the goal is reached and there are no other goals in the queue
                    self.zuuu_hal.mode = ZuuuModes.BRAKE
            else :
                self.get_logger().info(f"Aborting goal")
                goal_handle.abort()
                ret = "abort"
                self.nb_remaining_deletions -= 1
                
            # Populate result message
            result = ZuuuGoto.Result()
            result.result.status = ret

            self.execution_ongoing.set()
            return result
        except Exception as e:
            self.zuuu_hal.emergency_shutdown(f"Exception in goto_time: {e}")
    
    def setup_goto(self, goto_request):
        """Setup the goto with the parameters from the request"""
        # Note: the PID values have default values in the goto_request message
        self.keep_control_on_arrival = goto_request.keep_control_on_arrival
        self.dist_tol = goto_request.dist_tol
        self.angle_tol = goto_request.angle_tol
        # Most values are stored in the zuuu_hal object as they are used in the control loop
        self.zuuu_hal.distance_pid = PID(p=goto_request.distance_p, i=goto_request.distance_i, d=goto_request.distance_d, max_command=goto_request.distance_max_command, max_i_contribution=goto_request.distance_max_command/2.0)
        self.zuuu_hal.angle_pid = PID(p=goto_request.angle_p, i=goto_request.angle_i, d=goto_request.angle_d, max_command=goto_request.angle_max_command, max_i_contribution=goto_request.angle_max_command/2.0)
        self.zuuu_hal.distance_pid.set_goal(0.0)
        self.zuuu_hal.angle_pid.set_goal(0.0)
        # Setting the goal values in zuuu_hal also keeps compliance with e.g. DistanceToGoal service resquest
        self.zuuu_hal.x_goal = goto_request.x_goal
        self.zuuu_hal.y_goal = goto_request.y_goal
        self.zuuu_hal.theta_goal = goto_request.theta_goal
        # Setting zuuu_hal mode to GOTO
        if self.zuuu_hal.mode is not ZuuuModes.GOTO:
            self.get_logger().info(f"Switching from {self.zuuu_hal.mode} mode to {ZuuuModes.GOTO}.")
            self.zuuu_hal.mode = ZuuuModes.GOTO
            
    def set_goals_to_present_position(self):
        self.zuuu_hal.x_goal = self.zuuu_hal.x_odom
        self.zuuu_hal.y_goal = self.zuuu_hal.y_odom
        self.zuuu_hal.theta_goal = self.zuuu_hal.theta_odom
    
    def goto_time(self, goal_handle, goto_request):
        """Blocking function that sends commands to the mobile base to reach a goal pose in the odometry frame.
        This function will update the goal_handle state, publish feedback and return when the goal is reached or the timeout is reached
        or the goal is canceled by the client.
        """
        try:
            t0 = time.time()
            dt = 1 / self.sampling_freq
            timeout = goto_request.timeout
            commands_sent = 0
            while True:
                t0_loop = time.time()
                
                # Check for cancellation
                if goal_handle.is_cancel_requested :
                    self.set_goals_to_present_position()
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    return "canceled"

                # Timeout condition
                elapsed_time = time.time() - t0
                if elapsed_time > timeout:
                    self.get_logger().info(f"goto finished based on timeout")
                    self.set_goals_to_present_position()
                    goal_handle.abort()
                    return "timeout"
                
                if self.zuuu_hal.mode is ZuuuModes.GOTO :     
                    # The control calculations and speed commands updates are performed in the zuuu_hal control loop
                    arrived, distance_error, angle_error = self.check_goto_arrived()
                    if arrived:
                        goal_handle.succeed()
                        return "finished"
                else:
                    self.get_logger().info("Zuuu is not in GOTO mode anymore")
                    self.set_goals_to_present_position()
                    goal_handle.abort()
                    return "no_longer_goto_mode"

                # Publishing feedback every nb_commands_per_feedback ticks
                commands_sent += 1
                if commands_sent % self.nb_commands_per_feedback == 0:                
                    self.generate_and_publish_feedback(goal_handle, distance_error, angle_error)

                # Maintain loop frequency
                # self.rate.sleep()  # Slowly the output freq drops with this... A bug I could not find.
                time.sleep(max(0, dt - (time.time() - t0_loop)))
        except Exception as e:
            self.zuuu_hal.emergency_shutdown(f"Exception in goto_time: {e}")
            

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

    def check_goto_arrived(self):
        """Checks if the robot has reached the goal pose and provides the current distance and angle errors.
        """
        dx = self.zuuu_hal.x_odom - self.zuuu_hal.x_goal
        dy = self.zuuu_hal.y_odom - self.zuuu_hal.y_goal
        distance_error = math.sqrt(dx ** 2 + dy ** 2)

        angle_error = self.zuuu_hal.theta_odom - self.zuuu_hal.theta_goal
        arrived = False
        if distance_error < self.dist_tol and abs(angle_error) < self.angle_tol:
            self.get_logger().info("Reached the goal position !")
            arrived = True
        return arrived, distance_error, angle_error