#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""
import threading
from threading import Thread
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import os
import subprocess
import time

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

import math
import rosbag2_py


class EpisodeRecorder: 
    """Simple helper to start/stop rosbag2 recording.""" 
    def __init__(self, output_root="/home/bhavin-avina/avu_robot/episode_data"): 
        self.output_root = output_root 
        os.makedirs(self.output_root, exist_ok=True) 
        self._recorder = None 
        self._thread = None 
        self._recording = False 
        
    def start(self, topics=None): 
        if self._recording: 
            print("[Recorder] Already recording.") 
            return 
        ts = datetime.now().strftime("%Y%m%d_%H%M%S") 
        bag_dir = os.path.join(self.output_root, f"episode_{ts}") 
        print(f"[Recorder] Starting bag: {bag_dir}") 
        
        storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3') 
        record_options = rosbag2_py.RecordOptions() 
        record_options.all_topics = topics is None 
        record_options.topics = topics or [] 
        record_options.rmw_serialization_format = "cdr" 
        record_options.compression_mode = "" 
        record_options.compression_format = "" 
        record_options.include_unpublished_topics = False 
        record_options.ignore_leaf_topics = False 
        record_options.disable_keyboard_controls = True 

        self._recorder = rosbag2_py.Recorder() 
        self._thread = threading.Thread( target=self._recorder.record, args=(storage_options, record_options), daemon=True ) 
        self._thread.start() 
        self._recording = True 
        
    def stop(self): 
        if not self._recording: 
            print("[Recorder] No active recording to stop.") 
            return 
        print("[Recorder] Stopping...") 
        self._recorder.stop() 
        self._thread.join() 
        self._recording = False 
        print("[Recorder] Recording stopped.")
        

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")


        self.callback_group = ReentrantCallbackGroup()

        #block used only if recording data
        #self.recorder = EpisodeRecorder("/home/bhavin-avina/avu_robot/episode_data")
        #self.topics_to_record = [
        #    "/camera_front/image10",
        #    "/camera_top/image10",
        #    "/camera_wrist/image10",
        #    "/arm_controller/controller_state10",
        #    "/joint_states10",
        #]

        self.declare_parameter("target_color", "RED")
        self.target_color = self.get_parameter("target_color").value.upper()

        self.declare_parameter("target_shape", "SPHERE")
        self.target_shape = self.get_parameter("target_shape").value.upper()

        self.already_moved = False
        self.latest_target_coords = None


        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        self.sub = self.create_subscription(
            String, "/color_cord_and_shape", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")


        self.gripper_cmd_action_client = ActionClient(self, GripperCommand, "/gripper_action_controller/gripper_cmd")

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0)]
        
        self._start_timer = self.create_timer(0.5, self._start_sequence)
        self.get_logger().info("Pick and place node initialized.")

    def coords_callback(self, msg):
        if self.already_moved:
            return
        
        try:
            color_id, shape, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()
            shape = shape.strip().upper()

            if color_id == self.target_color and shape == self.target_shape:
                self.latest_target_coords = [float(x), float(y), float(z)]
                self.already_moved = True

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def _start_sequence(self):
        self._start_timer.cancel()
        Thread(target=self.run_sequence, daemon=True).start()

    def wait_for_gripper_server(self):
        self.get_logger().info("waiting for action gripper server")
        if not self.gripper_cmd_action_client.wait_for_server(timeout_sec=10):
            self.get_logger().info("gripper action server not available")
            return False
        self.get_logger().info("gripper_action server ready")
        return True
    
    def send_gripper_goal(self, position: float, effort: float=30.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = effort

        self.get_logger().info(f"sending gripper goal (position={position:.3f})")
        send_future = self.gripper_cmd_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("gripper goal rejected")
            return False
        
        self.get_logger().info("gripper goal accepted. waiting for result")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info("gripper action completed")
        return True
    
    def open_gripper(self):
        return self.send_gripper_goal(position=0.14, effort=30.0)
    
    def close_gripper(self):
        return self.send_gripper_goal(position=-0.5, effort=30.0)
    
    def run_sequence(self):
        self.get_logger().info("Starting pick-and-place sequence...")

        #self.recorder.start(self.topics_to_record)

        if not self.wait_for_gripper_server():
            self.get_logger().error("Aborting sequence: gripper server unavailable.")
            return

        if self.latest_target_coords is None:
            return
        
        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()


        pick_position = [self.latest_target_coords[0], self.latest_target_coords[1], self.latest_target_coords[2]-0.01]
        quat_xyzw = [-0.7071068, 0, 0, 0.7071068]
        drop_position = [0.123, -0.131, 0.19]
        quat_xyzw1 = [-0.7071068, 0, 0, 0.7071068]  

        self.open_gripper()
        self.get_logger().info("opened the gripper")

        #pick position
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw, tolerance_orientation=(0.005, 3.14, 0.005)) 
        self.moveit2.wait_until_executed() 
  
        # 4. Move down to approach object
        approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.01]
        self.moveit2.move_to_pose(position=approach_position, quat_xyzw=quat_xyzw, tolerance_orientation=(0.005, 3.14, 0.005), tolerance_position=0.001, cartesian=True
                                  )
        self.moveit2.wait_until_executed()
        self.get_logger().info(" at the pick position")

    

        # 5. Close gripper 
        self.close_gripper()
        self.get_logger().info("closed the gripper")


        # 8. Move to drop joint configuration
        self.moveit2.move_to_pose(position=drop_position, quat_xyzw=quat_xyzw1, tolerance_orientation=(0.9, 0.9, 0.9))
        self.moveit2.wait_until_executed()
        self.get_logger().info("moved above the drop position")

        # 9. Open gripper to release
        self.open_gripper()
        self.get_logger().info("dropped the object")


        # 11. Return to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()
        self.get_logger().info("moved to start position")

        self.get_logger().info("Pick-and-place sequence complete.")

        #self.recorder.stop()

        rclpy.shutdown()



def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()