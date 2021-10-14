#!/usr/bin/env python3
import numpy as np
import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

qos_profile = QoSProfile(depth=10)
qos_profile.history = QoSHistoryPolicy.KEEP_LAST
qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
qos_profile.durability = QoSDurabilityPolicy.VOLATILE


class PathProjection(Node):
    def __init__(self):
        super().__init__('path_projection_node')
        self.bridge = CvBridge()
        self.last_spline_coords = None
        self.odom_coords = np.zeros(shape=(0, 2))
        self.tf_coords = np.zeros(shape=(0, 2))
        self.stamp_memory = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0/30, self.on_timer)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/vehicle/odom_pose', self.odom_callback, qos_profile)
        self.paths_img_pub = self.create_publisher(Image, '/melex/path_projection/image', qos_profile)

    def odom_callback(self, msg):
        current_stamp = Time.from_msg(msg.header.stamp).nanoseconds

        if current_stamp < self.stamp_memory:
            self.odom_coords = np.zeros(shape=(0, 2))
            self.tf_coords = np.zeros(shape=(0, 2))
            self.tf_buffer.clear()

        self.stamp_memory = current_stamp

        self.odom_coords = np.append(self.odom_coords,
                                     np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]]),
                                     axis=0)
        self.plot_path()

    def plot_path(self):
        fig = Figure()
        canvas = FigureCanvas(fig)
        ax_odom = fig.gca()
        ax_tf = fig.gca()
        ax_odom.plot(self.odom_coords[:, 0], self.odom_coords[:, 1], label='odom')
        ax_tf.plot(self.tf_coords[:, 0], self.tf_coords[:, 1], label='tf')
        ax_odom.legend(loc='upper left')
        ax_tf.legend(loc='upper left')
        canvas.draw()
        image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(480, 640, 3)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            img_msg.header.frame_id = 'map'
            self.paths_img_pub.publish(img_msg)
        except (CvBridgeError, TypeError) as e:
            self.get_logger().warn(e)

    def on_timer(self):
        from_frame_rel = 'odom'
        to_frame_rel = 'map'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
            self.tf_coords = np.append(self.tf_coords,
                                         np.array([[trans.transform.translation.x, trans.transform.translation.y]]),
                                         axis=0)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


def main(args=None):
    rclpy.init(args=args)
    path_projection = PathProjection()
    rclpy.spin(path_projection)
    path_projection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
