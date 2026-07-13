import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException

from fastlio_adapter.settings import load_settings
from fastlio_adapter.transforms import (
    matrix_from_translation_quaternion,
    invert_transform,
    planar_pose,
    planar_twist,
    yaw_to_quaternion,
)


class OdometryAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_adapter_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings = load_settings(config_path)
        self.topics = self.settings.topics
        self.frames = self.settings.frames
        self.options = self.settings.options

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sensor_offset = None
        self.first_pose_inverse = None
        self.previous_pose = None
        self.transform_warning_logged = False

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        self.odometry_publisher = self.create_publisher(Odometry, self.topics.odometry_publisher, 10)
        self.create_subscription(Odometry, self.topics.odometry_subscriber, self.on_odometry, 10)

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================

    def lookup_sensor_offset(self):
        if self.sensor_offset is not None:
            return self.sensor_offset
        try:
            transform = self.tf_buffer.lookup_transform(self.frames.base, self.frames.sensor, rclpy.time.Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.sensor_offset = matrix_from_translation_quaternion(
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        return self.sensor_offset

    def on_odometry(self, message: Odometry):
        sensor_offset = self.lookup_sensor_offset()
        if sensor_offset is None:
            if not self.transform_warning_logged:
                self.get_logger().warning(
                    f"TF lookup {self.frames.base} -> {self.frames.sensor} failed in lookup_sensor_offset, "
                    f"odometry is not adapted until robot_state_publisher provides it"
                )
                self.transform_warning_logged = True
            return

        base_pose = self.base_pose_from_message(message, sensor_offset)
        x, y, yaw = planar_pose(base_pose)
        self.publish_odometry(x, y, yaw, message.header.stamp)

    def base_pose_from_message(self, message: Odometry, sensor_offset):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sensor_pose = matrix_from_translation_quaternion(
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        base_pose = sensor_pose @ invert_transform(sensor_offset)

        if self.options.zero_first_pose and self.first_pose_inverse is None:
            self.first_pose_inverse = invert_transform(base_pose)
        if self.first_pose_inverse is not None:
            base_pose = self.first_pose_inverse @ base_pose
        return base_pose

    def publish_odometry(self, x, y, yaw, stamp):
        current_time = stamp.sec + stamp.nanosec * 1e-9
        current_pose = (x, y, yaw, current_time)
        forward_velocity, angular_velocity = 0.0, 0.0
        if self.previous_pose is not None:
            forward_velocity, angular_velocity = planar_twist(self.previous_pose, current_pose)
        self.previous_pose = current_pose

        quaternion = yaw_to_quaternion(yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.frames.odom
        odometry.child_frame_id = self.frames.base
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.twist.twist.linear.x = forward_velocity
        odometry.twist.twist.angular.z = angular_velocity
        self.odometry_publisher.publish(odometry)

        if self.options.publish_tf:
            self.broadcast_transform(x, y, quaternion, stamp)

    def broadcast_transform(self, x, y, quaternion, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frames.odom
        transform.child_frame_id = self.frames.base
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
