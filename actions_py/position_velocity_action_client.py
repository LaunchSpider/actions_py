#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from custom_interfaces.action import PositionVelocity


class PositionVelocityActionClient(Node):
    def __init__(self):
        super().__init__("position_velocity_action_client")
        self.client_ = ActionClient(self, PositionVelocity, "position_velocity_action")

    def send_goal(self, position, velocity):
        self.client_.wait_for_server()
        goal = PositionVelocity.Goal()
        goal.position = position
        goal.velocity = velocity

        self.get_logger().info(f"Sending goal: Position={position}, Velocity={velocity}")
        self.client_.send_goal_async(goal, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        if self.goal_handle_:
            self.get_logger().info("Sending cancel request...")
            self.goal_handle_.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle_ = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal rejected.")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("Goal canceled.")
        self.get_logger().info(f"Result: {result.position}, Message: {result.message}")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: Current Position={feedback_msg.feedback.current_position}")


def main(args=None):
    rclpy.init(args=args)
    client = PositionVelocityActionClient()
    client.send_goal(110, 5)  # Example goal
    rclpy.spin(client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
