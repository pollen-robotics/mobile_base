import asyncio
import copy
from typing import List

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from zuuu_interfaces.action import ZuuuGoto


class ZuuuGotoActionClient(Node):
    def __init__(self):
        super().__init__("mobile_base_goto_action_client")
        self.goto_action_client = ActionClient(self, ZuuuGoto, "mobile_base_goto")
        self.get_logger().info(f"Waiting for action server mobile_base_goto...")
        self.goto_action_client.wait_for_server()

    def feedback_callback_default(self, feedback):
        self.get_logger().info(
            f"Received feedback. distance_error: {feedback.feedback.feedback.distance_error}, angle_error: {feedback.feedback.feedback.angle_error}"
        )

    async def send_goal(
        self,
        x_goal,
        y_goal,
        theta_goal,
        dist_tol,
        angle_tol,
        timeout,
        keep_control_on_arrival,
        distance_p,
        distance_i,
        distance_d,
        distance_max_command,
        angle_p,
        angle_i,
        angle_d,
        angle_max_command,
        feedback_callback=None,
        return_handle=False,
    ):
        goal_msg = ZuuuGoto.Goal()

        request = goal_msg.request  # This is of type zuuu_interfaces/ZuuuGotoRequest

        request.x_goal = x_goal
        request.y_goal = y_goal
        request.theta_goal = theta_goal
        request.dist_tol = dist_tol
        request.angle_tol = angle_tol
        request.timeout = timeout
        request.keep_control_on_arrival = keep_control_on_arrival
        request.distance_p = distance_p
        request.distance_i = distance_i
        request.distance_d = distance_d
        request.distance_max_command = distance_max_command
        request.angle_p = angle_p
        request.angle_i = angle_i
        request.angle_d = angle_d
        request.angle_max_command = angle_max_command

        self.get_logger().warning(f"Sending zuuu goto goal request: {request}")

        goal_handle = await self.goto_action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        self.get_logger().info("feedback_callback setuped")

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")

        if return_handle:
            return goal_handle
        else:
            res = await goal_handle.get_result_async()
            result = res.result
            status = res.status
            self.get_logger().info(f"Goto finished. Result: {result.result.status}")
            return result, status


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def blocking_demo(action_client):
    logger = rclpy.logging.get_logger("mobile_base_goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 1: blocking calls")
    # Setting the feedback callback only once because it's very verbose
    result, status = await action_client.send_goal(
        0.0,
        0.0,
        np.deg2rad(720.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )
    result, status = await action_client.send_goal(
        1.0,
        0.0,
        0.0,
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )

    # An example on how to read result and status:
    if status == GoalStatus.STATUS_SUCCEEDED:
        logger.info(f"Goal succeeded! Result: {result.result.status}")
    else:
        logger.info(f"Goal failed. Result: {result.result.status}")

    result, status = await action_client.send_goal(
        0.0,
        0.0,
        0.0,
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )

    # Sleeping in place can lead to a better precision than the tolerances set
    await asyncio.sleep(1.0)

    result, status = await action_client.send_goal(
        0.0,
        0.0,
        np.deg2rad(45),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )

    await asyncio.sleep(1.0)

    result, status = await action_client.send_goal(
        1.0,
        1.0,
        np.deg2rad(45),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )

    result, status = await action_client.send_goal(
        0.0,
        0.0,
        np.deg2rad(0.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        feedback_callback=action_client.feedback_callback_default,
    )


async def non_blocking_demo(action_client, loop):
    logger = rclpy.logging.get_logger("mobile_base_goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 2: simultaneous async calls")
    my_task1 = loop.create_task(
        action_client.send_goal(
            1.0,
            1.0,
            np.deg2rad(45),
            0.05,
            np.deg2rad(5),
            10.0,
            True,
            5.0,
            0.0,
            0.0,
            0.4,
            5.0,
            0.0,
            0.0,
            1.0,
            feedback_callback=action_client.feedback_callback_default,
        )
    )
    my_task2 = loop.create_task(
        action_client.send_goal(
            1.0,
            2.0,
            np.deg2rad(45),
            0.05,
            np.deg2rad(5),
            10.0,
            True,
            5.0,
            0.0,
            0.0,
            0.4,
            5.0,
            0.0,
            0.0,
            1.0,
            feedback_callback=action_client.feedback_callback_default,
        )
    )

    my_task3 = loop.create_task(
        action_client.send_goal(
            1.0,
            2.0,
            np.deg2rad(180.0),
            0.05,
            np.deg2rad(5),
            10.0,
            True,
            5.0,
            0.0,
            0.0,
            0.4,
            5.0,
            0.0,
            0.0,
            1.0,
            feedback_callback=action_client.feedback_callback_default,
        )
    )

    logger.info(f"Gluing tasks and waiting")

    wait_future = asyncio.wait([my_task1, my_task2, my_task3])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")

    logger.info(f"$$$$$$ EXAMPLE 2: Going back to start position")

    my_task1 = loop.create_task(
        action_client.send_goal(
            0.0,
            0.0,
            np.deg2rad(180.0),
            0.05,
            np.deg2rad(5),
            10.0,
            True,
            5.0,
            0.0,
            0.0,
            0.4,
            5.0,
            0.0,
            0.0,
            1.0,
            feedback_callback=action_client.feedback_callback_default,
        )
    )
    my_task2 = loop.create_task(
        action_client.send_goal(
            0.0,
            0.0,
            np.deg2rad(0.0),
            0.05,
            np.deg2rad(5),
            10.0,
            True,
            5.0,
            0.0,
            0.0,
            0.4,
            5.0,
            0.0,
            0.0,
            1.0,
            feedback_callback=action_client.feedback_callback_default,
        )
    )

    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")


async def square_demo(action_client, max_speed, max_rotation_speed):
    logger = rclpy.logging.get_logger("mobile_base_goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 4: square demo max_speed: {max_speed}, max_rotation_speed: {max_rotation_speed}")
    result, status = await action_client.send_goal(
        1.0,
        0.0,
        np.deg2rad(0.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        max_speed,
        5.0,
        0.0,
        0.0,
        max_rotation_speed,
        feedback_callback=action_client.feedback_callback_default,
    )
    result, status = await action_client.send_goal(
        1.0,
        1.0,
        np.deg2rad(0.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        max_speed,
        5.0,
        0.0,
        0.0,
        max_rotation_speed,
        feedback_callback=action_client.feedback_callback_default,
    )
    result, status = await action_client.send_goal(
        0.0,
        1.0,
        np.deg2rad(0.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        max_speed,
        5.0,
        0.0,
        0.0,
        max_rotation_speed,
        feedback_callback=action_client.feedback_callback_default,
    )
    result, status = await action_client.send_goal(
        0.0,
        0.0,
        np.deg2rad(0.0),
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        max_speed,
        5.0,
        0.0,
        0.0,
        max_rotation_speed,
        feedback_callback=action_client.feedback_callback_default,
    )


async def cancel_demo(action_client, loop):
    logger = rclpy.logging.get_logger("mobile_base_goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 3: cancel demo")

    goal_handle = await action_client.send_goal(
        1.0,
        0.0,
        0.0,
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        return_handle=True,
        feedback_callback=action_client.feedback_callback_default,
    )

    await asyncio.sleep(1.0)
    await goal_handle.cancel_goal_async()
    logger.info(f"Goal canceled!")
    await asyncio.sleep(3.0)

    logger.info(f"Going back to 0")
    await action_client.send_goal(
        0.0,
        0.0,
        0.0,
        0.05,
        np.deg2rad(5),
        10.0,
        True,
        5.0,
        0.0,
        0.0,
        0.4,
        5.0,
        0.0,
        0.0,
        1.0,
        return_handle=False,
    )


async def run_demo(args, loop):
    rclpy.init(args=args)

    # create node
    action_client = ZuuuGotoActionClient()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    # # Demo 1: blocking calls
    await blocking_demo(action_client)

    # # Demo 2: non-blocking calls called simultaneously
    await non_blocking_demo(action_client, loop)

    # # Demo 3: cancel
    await cancel_demo(action_client, loop)

    # # Demo 4: square with different speeds
    await square_demo(action_client, 0.4, 1.0)
    await square_demo(action_client, 0.2, 0.5)
    await square_demo(action_client, 0.8, 2.0)

    # cancel spinning task
    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_demo(args, loop=loop))
    # done, _pending = loop.run_until_complete(run_demo(args, loop=loop))

    # for task in done:
    #     task.result()


if __name__ == "__main__":
    main()
