#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf_transformations
import cv2
import numpy as np
from rclpy.duration import Duration
import struct
import math



class ShapeandColorExtractor(Node):
    def __init__(self):
        super().__init__("shape_and_color_extractor")

        self.image_sub_ = self.create_subscription(Image, "/camera_top/image", self.im_cb, 10)
        self.pc_sub_ = self.create_subscription(PointCloud2, "/camera_top/points", self.pc_cb, 10)

        self.pub_ = self.create_publisher(String, "color_cord_and_shape", 10)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #intrinsic of the camera
        self.fx = 443.5
        self.fy = 443.5
        self.cx = 320.0
        self.cy = 240.0

        self.latest_rgb = None        # cv2 image (bgr)
        self.latest_pc = None 

        self.get_logger().info("Color Detector Node Started with TF2 lookup transform")

    def pc_cb(self, msg: PointCloud2):
        # store the latest pointcloud message
        self.latest_pc = msg

    def im_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.latest_rgb = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY)

        color_ranges = {
            "RED": [(0, 60, 50), (10, 255, 255)],
            "BLUE": [(90, 60, 50), (140, 255, 255)]
        }

        for color_id, (lower, upper) in color_ranges.items():
            upper = np.array(upper)
            lower = np.array(lower)
            mask = cv2.inRange(hsv, lower, upper)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 1:
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                    shape = "unidentified"
                    vertices = len(approx)
                    x, y, w, h = cv2.boundingRect(cnt)
                    M = cv2.moments(cnt)
                    if M["m00"] == 0:
                        continue
                    cx_pix = int(M["m10"] / M["m00"])
                    cy_pix = int(M["m01"] / M["m00"])

                    self.get_logger().info(f"{cx_pix}, {cy_pix}")

                    if vertices == 4:
                        shape = "CUBE"

                    else:
                        shape = "SPHERE"

                    cv2.putText(frame, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    cv2.putText(frame, color_id, (x, y - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.drawContours(frame, [cnt], -1, (0,255,0), 2)

                    pts = []
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            u = cx_pix + dx
                            v = cy_pix + dy
                            p = self.get_point_from_latest_cloud(u, v)
                            if p is not None:
                                pts.append(p)

                    if len(pts) == 0:
                        self.get_logger().warn("No valid depth near centroid")
                        continue

                    X, Y, Z = np.mean(pts, axis=0)

                    try:
                        # Lookup transform camera_link -> panda_link0
                        # Use Time(seconds=0) for latest available transform
                        t = self.tf_buffer.lookup_transform(
                            "base_link", 
                            "camera_top_link", 
                            rclpy.time.Time(),
                            timeout=Duration(seconds=1.0))

                        # Convert to numpy transform matrix
                        trans = np.array([
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z
                        ])
                        
                        rot = [
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w
                        ]
                        
                        # Create 4x4 transformation matrix
                        T = tf_transformations.quaternion_matrix(rot)
                        T[:3, 3] = trans

                        # Transform point from camera frame to base frame
                        pt_cam = np.array([X, Y, Z, 1.0])
                        pt_base = T @ pt_cam

                        msg_str = f"{color_id},{shape},{pt_base[0]:.3f},{pt_base[1]:.3f},{pt_base[2]:.3f}"
                        self.pub_.publish(String(data=msg_str))
                        self.get_logger().info(msg_str)

                    
                    except (tf2_ros.LookupException, 
                            tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"TF lookup failed: {e}")

                    except Exception as e:
                        self.get_logger().error(f"Unexpected error in TF transform: {e}")

        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 480)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"OpenCV display error: {e}")


    def get_point_from_latest_cloud(self, u: int, v: int):
        """
        Return (x, y, z) in camera frame for pixel (u,v) using the latest PointCloud2 message.
        Returns None if no valid point or pointcloud not available / not organized.
        """
        pc = self.latest_pc
        if pc is None:
            self.get_logger().debug("No pointcloud received yet.")
            return None

        # verify that pointcloud is organized (width x height == num points and height > 1)
        if pc.height == 1:
            # unorganized cloud (height == 1): cannot index by (u,v) reliably
            self.get_logger().warn("PointCloud2 appears unorganized (height==1). Cannot index by pixel. "
                                   "Ensure you publish an organized point cloud (same width/height as RGB).")
            return None

        # ensure u,v inside bounds
        if u < 0 or v < 0 or u >= pc.width or v >= pc.height:
            self.get_logger().warn(f"Pixel ({u},{v}) out of bounds for pointcloud (w={pc.width}, h={pc.height})")
            return None

        idx = v * pc.width + u
        point_step = pc.point_step
        offset = idx * point_step

        # find offsets for x,y,z fields
        offs = {'x': None, 'y': None, 'z': None}
        for field in pc.fields:
            if field.name in offs:
                offs[field.name] = field.offset

        if offs['x'] is None or offs['y'] is None or offs['z'] is None:
            self.get_logger().error("PointCloud2 missing x/y/z fields.")
            return None

        # pick endianness
        endian_char = '>' if pc.is_bigendian else '<'

        try:
            # unpack floats from the raw data buffer
            raw = pc.data  # bytes
            # offsets are relative to the start of the point (so add base offset)
            x = struct.unpack_from(endian_char + 'f', raw, offset + offs['x'])[0]
            y = struct.unpack_from(endian_char + 'f', raw, offset + offs['y'])[0]
            z = struct.unpack_from(endian_char + 'f', raw, offset + offs['z'])[0]
            # check for invalid values (common in depth cameras)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return None
            # some sensors use 0 to mean invalid
            if z == 0.0:
                return None
            return (float(x), float(y), float(z))
        except struct.error as e:
            self.get_logger().error(f"Error unpacking pointcloud data: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading pointcloud: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ShapeandColorExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()