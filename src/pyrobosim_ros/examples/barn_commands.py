#!/usr/bin/env python3

"""
Example showing how to request task actions and plans.
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
import time

from pyrobosim_msgs.action import ExecuteTaskAction, ExecuteTaskPlan  # type: ignore
from pyrobosim_msgs.msg import TaskAction, TaskPlan  # type: ignore
from pyrobosim_msgs.srv import RequestWorldState  # type: ignore


class Commander(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("demo_commander")

        # Declare node parameters
        self.declare_parameter("mode", value="plan")
        self.declare_parameter("send_cancel", value=False)

        # Action client for a single action
        self.action_client = ActionClient(self, ExecuteTaskAction, "execute_action")

        # Action client for a task plan
        self.plan_client = ActionClient(self, ExecuteTaskPlan, "execute_task_plan")

        # Call world state service to ensure node is running
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state"
        )
        while rclpy.ok() and not self.world_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for world state server...")
        future = self.world_state_client.call_async(RequestWorldState.Request())
        rclpy.spin_until_future_complete(self, future)

    def send_action_goal(
        self, goal: ExecuteTaskAction.Goal, cancel: bool = False
    ) -> None:
        self.action_client.wait_for_server()
        goal_future = self.action_client.send_goal_async(goal)
        if cancel:
            goal_future.add_done_callback(self.goal_response_callback)

    def send_plan_goal(self, goal: ExecuteTaskPlan.Goal, cancel: bool = False) -> None:
        self.plan_client.wait_for_server()
        goal_future = self.plan_client.send_goal_async(goal)
        if cancel:
            goal_future.add_done_callback(
                lambda goal_future: self.goal_response_callback(
                    goal_future, cancel_delay=12.5
                )
            )

    def goal_response_callback(self, goal_future: Future, cancel_delay: float = 2.0) -> None:  # type: ignore[type-arg] # Cannot add args in Humble and Jazzy
        """Starts a timer to cancel the goal handle, upon receiving an accepted goal."""
        goal_handle = goal_future.result()
        if goal_handle is None:
            self.get_logger().error("Goal handle is None.")
            return
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return

        self.cancel_timer = self.create_timer(
            cancel_delay, lambda: self.cancel_goal(goal_handle)
        )

    def cancel_goal(self, goal: ExecuteTaskAction.Goal | ExecuteTaskPlan.Goal) -> None:
        """Timer callback function that cancels a goal."""
        self.get_logger().info("Canceling goal")
        goal.cancel_goal_async()
        self.cancel_timer.cancel()


def main() -> None:
    rclpy.init()
    cmd = Commander()

    # Choose between action or plan command, based on input parameter.
    mode = cmd.get_parameter("mode").value
    send_cancel = cmd.get_parameter("send_cancel").value

    # Valid locations
    location_list = ["dock", "3dprinters", "desktops", "class", "cricut",
                     "solder", "xtool", "red", "blue"]
    
    if mode in location_list:
        cmd.get_logger().info("Executing action " + mode + " ...")
        goal = ExecuteTaskAction.Goal()
        goal.action = TaskAction(
            robot="robot", type="navigate", target_location=mode,
        )
        cmd.send_action_goal(goal, cancel=send_cancel)

    elif mode == "plan":
        cmd.get_logger().info("Executing tour ...")
        task_actions = []
        for item in location_list:
            task_actions.append(TaskAction(type="navigate", target_location=item))
        goal = ExecuteTaskPlan.Goal()
        goal.plan = TaskPlan(robot="robot", actions=task_actions)
        cmd.send_plan_goal(goal, cancel=send_cancel)

    else:
        cmd.get_logger().error(f"Invalid mode specified: {mode}")

    cmd.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
