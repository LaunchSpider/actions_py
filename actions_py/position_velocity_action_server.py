#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_interfaces.action import PositionVelocity


class PositionVelocityActionServer(Node):
    def __init__(self):
        super().__init__("position_velocity_action_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue = []
        self.current_position = 0  # Save position across goals
        self.server_ = ActionServer(
            self,
            PositionVelocity,
            "position_velocity_action",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("PositionVelocity Action Server has been started...")

    def goal_callback(self, goal_request: PositionVelocity.Goal):
        self.get_logger().info(f"Received a goal: Position={goal_request.position}, Velocity={goal_request.velocity}")

        # Check if the position is within the valid range (0 to 100)
        if not (0 <= goal_request.position <= 100):
            self.get_logger().warn(f"Rejected goal: Position {goal_request.position} out of range. It must be between 0 and 100.")
            return GoalResponse.REJECT
        
        # Check if the velocity is non-negative
        if goal_request.velocity < 0:
            self.get_logger().warn(f"Rejected goal: Velocity {goal_request.velocity} is negative. It must be non-negative.")
            return GoalResponse.REJECT

        # Preempt the current goal if one is active
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Preempting current goal.")
                self.goal_handle_.abort()

        # Accept the new goal
        self.get_logger().info("Accepting the new goal.")
        return GoalResponse.ACCEPT


    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Cancel request received.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        position_target = goal_handle.request.position
        velocity = goal_handle.request.velocity
        feedback = PositionVelocity.Feedback()
        result = PositionVelocity.Result()

        self.get_logger().info("Executing goal...")

        # Determine the direction based on target position
        if self.current_position < position_target:
            # Moving forward (increment)
            direction = 1
        else:
            # Moving backward (decrement)
            direction = -1

        while self.current_position != position_target:
            # Handle cancellation
            if not goal_handle.is_active:
                self.get_logger().info("Goal is no longer active. Preempting...")
                result.position = self.current_position
                result.message = "Goal preempted."
                self.process_next_goal_in_queue()
                return result

            # Increment or decrement position based on the direction
            self.current_position += direction * velocity

            # Clamp the position to the target if it overshoots
            if direction == 1 and self.current_position > position_target:
                self.current_position = position_target
            elif direction == -1 and self.current_position < position_target:
                self.current_position = position_target

            # Publish feedback
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"Current position: {self.current_position}")
            time.sleep(1)

        # Goal succeeded
        self.get_logger().info("Goal completed successfully.")
        goal_handle.succeed()
        result.position = self.current_position
        result.message = "Goal completed successfully."
        self.process_next_goal_in_queue()
        return result


    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if self.goal_queue:
                self.goal_queue.pop(0).execute()
            else:
                self.goal_handle_ = None


def main(args=None):
    rclpy.init(args=args)
    node = PositionVelocityActionServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
