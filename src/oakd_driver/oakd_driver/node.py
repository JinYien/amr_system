import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from custom_message.msg import Control, Detection, DetectionArray, Sound
from oakd_driver.model import load_model_config
from oakd_driver.pipeline import build_pipeline, latest_packet, slow_usb_speed
from oakd_driver.settings import Settings, load_settings
from oakd_driver.visualization import Annotation, draw_annotations, encode_jpeg


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config_path)

        self.model = load_model_config(self.settings.model)
        self.confidence_threshold = self.settings.model.confidence_threshold
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.latest_frame = None
        self.last_image_time = None

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        topics = self.settings.topics
        qos = qos_profile_sensor_data

        self.detection_publisher = self.create_publisher(DetectionArray, topics.detection_publisher, qos)
        self.sound_publisher = None
        if self.settings.sounds.enable:
            self.sound_publisher = self.create_publisher(Sound, topics.sound_publisher, qos)
        self.mode = "manual"
        self.create_subscription(Control, topics.control_subscriber, self.control_callback, qos)
        self.detection_seen = {}
        self.image_publisher = None
        self.compressed_image_publisher = None
        if self.settings.visualization.enable:
            self.image_publisher = self.create_publisher(Image, topics.image_publisher, qos)
            self.compressed_image_publisher = self.create_publisher(
                CompressedImage, topics.image_publisher + "/compressed", qos
            )

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================
        self.create_timer(self.settings.device.reconnect_delay, self.connect_loop)
        self.create_timer(1.0 / (2.0 * self.settings.device.fps), self.poll_loop)

    def connect_loop(self):
        if self.pipeline is not None:
            return
        try:
            pipeline, detection_queue, preview_queue = build_pipeline(self.settings, self.model, self.confidence_threshold)
            pipeline.start()
            self.pipeline = pipeline
            self.detection_queue = detection_queue
            self.preview_queue = preview_queue

            slow = slow_usb_speed(pipeline)
            if slow is not None:
                self.get_logger().warning(f"Camera is on a slow USB port ({slow}); the frame rate will be lower than normal")
        except Exception as error:
            self.get_logger().warning(
                f"Camera connect failed in build_pipeline, retrying every {self.settings.device.reconnect_delay}s "
                f"({type(error).__name__}: {error})"
            )
            self.disconnect()

    def disconnect(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.latest_frame = None

    def poll_loop(self):
        if self.pipeline is None:
            return
        try:
            if not self.pipeline.isRunning():
                raise RuntimeError("Pipeline stopped")

            if self.preview_queue is not None:
                frame_packet = latest_packet(self.preview_queue)
                if frame_packet is not None:
                    self.latest_frame = frame_packet.getCvFrame()

            detection_packet = latest_packet(self.detection_queue)
            if detection_packet is not None:
                self.publish_detections(detection_packet.detections)
        except Exception as error:
            self.get_logger().error(
                f"Camera connection lost while polling queues in poll_loop, reconnecting "
                f"({type(error).__name__}: {error})"
            )
            self.disconnect()

    def image_subscriber_count(self) -> int:
        count = 0
        if self.image_publisher is not None:
            count += self.image_publisher.get_subscription_count()
        if self.compressed_image_publisher is not None:
            count += self.compressed_image_publisher.get_subscription_count()
        return count

    def publish_detections(self, detections):
        message = DetectionArray()
        visualize = self.image_subscriber_count() > 0
        annotations = []
        labels = set()
        for detection in detections:
            label = detection.labelName or self.model.label_name(detection.label)
            labels.add(label)
            result = Detection()
            result.label = label
            message.detections.append(result)
            if visualize:
                annotations.append(
                    Annotation(
                        label=label,
                        confidence=float(detection.confidence),
                        box=(float(detection.xmin), float(detection.ymin), float(detection.xmax), float(detection.ymax)),
                    )
                )
        self.detection_publisher.publish(message)
        self.announce_detections(labels)
        if visualize:
            self.publish_image(annotations)

    def control_callback(self, message: Control):
        self.mode = message.mode

    def announce_detections(self, labels):
        if self.sound_publisher is None or self.mode != "auto":
            return
        now = self.get_clock().now().nanoseconds / 1e9
        for label in labels:
            last_seen = self.detection_seen.get(label)
            if last_seen is None or now - last_seen > self.settings.sounds.forget_time:
                self.sound_publisher.publish(Sound(object=label))
            self.detection_seen[label] = now

    def publish_image(self, annotations):
        if self.latest_frame is None:
            return

        now = self.get_clock().now()
        if self.last_image_time is not None:
            elapsed = (now - self.last_image_time).nanoseconds / 1e9
            if elapsed < 1.0 / self.settings.visualization.publish_rate:
                return
        self.last_image_time = now

        frame = self.latest_frame
        if annotations:
            frame = draw_annotations(frame.copy(), annotations)
        height, width = frame.shape[:2]
        stamp = now.to_msg()

        if self.image_publisher.get_subscription_count() > 0:
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = "oakd"
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = 0
            image.step = width * 3
            image.data = frame.tobytes()
            self.image_publisher.publish(image)

        if self.compressed_image_publisher.get_subscription_count() > 0:
            data = encode_jpeg(frame, self.settings.visualization.jpeg_quality)
            if data is not None:
                compressed = CompressedImage()
                compressed.header.stamp = stamp
                compressed.header.frame_id = "oakd"
                compressed.format = "jpeg"
                compressed.data = data
                self.compressed_image_publisher.publish(compressed)

    def destroy_node(self):
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
