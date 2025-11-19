import rosbag2_py
import rclpy.serialization
from sensor_msgs.msg import JointState

bag_path = "/home/bhavin-avina/avu_robot/episode_data/episode_20251110_153656"

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/joint_states10":
        msg = rclpy.serialization.deserialize_message(data, JointState)
        print(f"Time: {t}")
        print(f"Names: {msg.name}")
        print(f"Positions: {msg.position}")
        print(f"Velocities: {msg.velocity}")
        print(f"Efforts: {msg.effort}")
        print("-" * 40)
