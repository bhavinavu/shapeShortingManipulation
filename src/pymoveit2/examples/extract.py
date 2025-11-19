#!/usr/bin/env python3
"""
use to Extract Pi0-style dataset from multiple ros2 .db3 episodes.

Usage:
    python3 extract_pi0_dataset.py

Dependencies:
    - python3-rosbag2 (rosbag2_py)
    - rclpy (for serialization)
    - cv_bridge
    - sensor_msgs, control_msgs
    - opencv-python
"""

import os
import json
import cv2
import bisect
import argparse
import numpy as np

import rclpy.serialization
import rosbag2_py

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from control_msgs.msg import JointTrajectoryControllerState


bridge = CvBridge()



def process_episode(bag_dir, output_root):

    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)

    # --- LOAD ALL MESSAGES ---
    messages = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        messages.append((topic, data, t))

    # --- ROUND DOWN TO MULTIPLE OF 5 ---
    N = len(messages)
    N_consider = N - (N % 5)      # ensures 5 messages per frame
    messages = messages[:N_consider]

    # Create output folder
    ep_id = os.path.basename(bag_dir.rstrip('/'))
    ep_out = os.path.join(output_root, ep_id)
    os.makedirs(ep_out, exist_ok=True)

    # --- PROCESS FRAMES ---
    frame_idx = 0

    for i in range(0, N_consider, 5):

        frame_msgs = messages[i:i+5]      # 5 topics = 1 frame
        frame_time = round(frame_idx * 0.1, 3)  # 0.0, 0.1, 0.2, ...

        frame_data = {
            "timestamp": frame_time,
            "wrist_image": None,
            "top_image": None,
            "front_image": None,
            "state": None,
            "action": None,
        }

        # Extract topics from these 5 messages
        for (topic, data, t) in frame_msgs:

            # IMAGE TOPIC
            if topic == "/camera_wrist/image10":
                img = rclpy.serialization.deserialize_message(data, Image)
                cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
                path = os.path.join(ep_out, f"frame_{frame_idx:06d}_wrist.jpg")
                cv2.imwrite(path, cv_img)
                frame_data["wrist_image"] = path

            elif topic == "/camera_top/image10":
                img = rclpy.serialization.deserialize_message(data, Image)
                cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
                path = os.path.join(ep_out, f"frame_{frame_idx:06d}_top.jpg")
                cv2.imwrite(path, cv_img)
                frame_data["top_image"] = path

            elif topic == "/camera_front/image10":
                img = rclpy.serialization.deserialize_message(data, Image)
                cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
                path = os.path.join(ep_out, f"frame_{frame_idx:06d}_front.jpg")
                cv2.imwrite(path, cv_img)
                frame_data["front_image"] = path

            # JOINT STATE TOPIC
            elif topic == "/joint_states10":
                js = rclpy.serialization.deserialize_message(data, JointState)
                
                frame_data["state"] = list(js.position)

            # CONTROLLER TOPIC
            elif topic == "/arm_controller/controller_state10":
                ctrl = rclpy.serialization.deserialize_message(
                    data, JointTrajectoryControllerState
                )
                frame_data["action"] = list(ctrl.reference.positions)

            # OTHER TOPICS
            else:
                pass

        if (frame_data["wrist_image"] is None and
            frame_data["top_image"] is None and
            frame_data["front_image"] is None and
            frame_data["state"] is None and
            frame_data["action"] is None):
            continue

        # Save JSON metadata
        json_path = os.path.join(ep_out, f"frame_{frame_idx:06d}.json")
        with open(json_path, "w") as f:
            json.dump(frame_data, f, indent=4)

        frame_idx += 1

    print(f"Episode {ep_id}: {frame_idx} frames")
    return frame_idx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode_root", type=str, default="episode_data1",
                        help="Root folder containing episode subfolders (each a rosbag2 sqlite3 folder).")
    parser.add_argument("--output_root", type=str, default="pi0_dataset")
    args = parser.parse_args()


    os.makedirs(args.output_root, exist_ok=True)

    total = 0
    episodes = sorted([d for d in os.listdir(args.episode_root) if os.path.isdir(os.path.join(args.episode_root, d))])
    for ep in episodes:
        ep_dir = os.path.join(args.episode_root, ep)
        try:
            n = process_episode(ep_dir, args.output_root)
            total += n
        except Exception as e:
            print(f"ERROR processing {ep_dir}: {e}")

    print(f"\nDone. Total frames extracted: {total}")
    print(f"Dataset root: {args.output_root}")

if __name__ == "__main__":
    main()



        






    