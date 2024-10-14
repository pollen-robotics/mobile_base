import threading
import time
from queue import Queue
from threading import Event
import math

import numpy as np
import rclpy
from zuuu_description.action import ZuuuGoto
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
)  # ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from zuuu_hal.zuuu_hal import ZuuuModes

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
        start_time = time.time()
        ret = ""

        goto_request = goal_handle.request.request  # zuuu_interfaces/ZuuuGotoRequest

        angle_tol = goto_request.angle_tol
        
        self.get_logger().info(f"Executing goal: {goto_request}")
        
        ret = self.goto_time(goal_handle, goto_request)

        if ret == "finished":
            goal_handle.succeed()

        # Populate result message
        result = ZuuuGoto.Result()
        result.result.status = ret
        self.get_logger().debug(f"Returning result {result}")

        self.get_logger().debug(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms at the end"
        )
        self.execution_ongoing.set()
        return result
    
    def goto_time(self, goal_handle, goto_request):
        t0 = time.time()
        dt = 1 / self.sampling_freq
        timeout = goto_request.timeout
        x_goal = goto_request.x_goal
        y_goal = goto_request.y_goal
        theta_goal = goto_request.theta_goal
        dist_tol = goto_request.dist_tol

        commands_sent = 0
        while True:
            t0_loop = time.time()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return "canceled"

            elapsed_time = time.time() - t0
            if elapsed_time > timeout:
                self.get_logger().info(f"goto finished based on timeout")
                break

            point = traj_func(elapsed_time)

            self.cmd_pub(joints, point)
            commands_sent += 1

            if commands_sent % self.nb_commands_per_feedback == 0:
                feedback_msg = ZuuuGoto.Feedback()
                feedback_msg.feedback.status = "running"
                feedback_msg.feedback.commands_sent = commands_sent
                feedback_msg.feedback.time_to_completion = timeout - elapsed_time
                goal_handle.publish_feedback(feedback_msg)

            # self.rate.sleep()  # Slowly the output freq drops with this...
            time.sleep(max(0, dt - (time.time() - t0_loop)))

        return "finished"

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f"Received cancel request")

        # Check state and decide
        if goal_handle.is_active or goal_handle.is_executing:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    
    def goto_tick(self)-> None:
        """Tick function for the goto mode. Will calculate the robot's speed to reach a goal pose in the odometry frame.
        """
        if self.zuuu_hal.mode is ZuuuModes.GOTO :
            distance = math.sqrt((self.x_goal - self.x_odom) ** 2 + (self.y_goal - self.y_odom) ** 2)
            if distance < self.xy_tol and abs(angle_diff(self.theta_goal, self.theta_odom)) < self.theta_tol:
                self.goto_service_on = False
                self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal =  0.0, 0.0, 0.0
                self.get_logger().info("Reached the goal position !")
            else:
                self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal = self.position_control()
        else:
            self.x_vel_goal, self.y_vel_goal, self.theta_vel_goal =  0.0, 0.0, 0.0
