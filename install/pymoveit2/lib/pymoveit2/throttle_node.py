#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

class ThrottleNode(Node):
    def __init__(self):
        super().__init__("frequency_throttler")

        self.image_top_sub_ = self.create_subscription(Image, "/camera_top/image", self.top_im_cb, 10)
        self.image_front_sub_ = self.create_subscription(Image, "/camera_front/image", self.front_im_cb, 10)
        self.image_wrist_sub_ = self.create_subscription(Image, "/camera_wrist/image", self.wrist_im_cb, 10)
        self.joint_states_sub_ = self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)
        self.controller_state_sub_ = self.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state", self.controller_state_cb, 10)


        self.image_top_pub_ = self.create_publisher(Image, "/camera_top/image10", 10)
        self.image_front_pub_ = self.create_publisher(Image, "/camera_front/image10", 10)
        self.image_wrist_pub_ = self.create_publisher(Image, "/camera_wrist/image10", 10)
        self.joint_states_pub_ = self.create_publisher(JointState, "/joint_states10", 10)
        self.controller_state_pub_ = self.create_publisher(JointTrajectoryControllerState, "/arm_controller/controller_state10", 10)

        self.last_time_top = self.get_clock().now()
        self.last_time_front = self.get_clock().now()
        self.last_time_wrist = self.get_clock().now()
        self.last_time_joint = self.get_clock().now()
        self.last_time_ctrl = self.get_clock().now()

    def top_im_cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_time_top).nanoseconds * 1e-9 >= 0.1:
            self.image_top_pub_.publish(msg)
            self.last_time_top = now

    def front_im_cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_time_front).nanoseconds * 1e-9 >= 0.1:
            self.image_front_pub_.publish(msg)
            self.last_time_front = now

    def wrist_im_cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_time_wrist).nanoseconds * 1e-9 >= 0.1:
            self.image_wrist_pub_.publish(msg)
            self.last_time_wrist = now

    def joint_states_cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_time_joint).nanoseconds * 1e-9 >= 0.1:
            self.joint_states_pub_.publish(msg)
            self.last_time_joint = now

    def controller_state_cb(self, msg):
        now = self.get_clock().now()
        if (now - self.last_time_ctrl).nanoseconds * 1e-9 >= 0.1:
            self.controller_state_pub_.publish(msg)
            self.last_time_ctrl = now


def main():
    rclpy.init()
    node = ThrottleNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()