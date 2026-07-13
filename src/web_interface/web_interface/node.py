import math
import threading
import traceback
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from custom_message.msg import JoystickCommand, Control, Sound
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from web_interface.assets import find_sound_names, find_pages_directory, load_navigation_map
from web_interface.logger import TopicLogger
from web_interface.teleop import Teleop
from web_interface.server import start_web_server
from web_interface.settings import load_settings

NAVIGATION_AUTHORITIES = ("robot", "mix")


class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__("web_interface_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings = load_settings(config_path)

        self.teleop = Teleop()
        self.teleop_lock = threading.Lock()
        self.send_to_browser = None
        self.sound_names = ["beep", "chime"] + find_sound_names()

        self.browser_event_handlers = {
            "axes": self.handle_joystick,
            "mode": self.handle_mode,
            "authority": self.handle_authority,
            "middle": self.handle_middle,
            "sounds_request": self.handle_sounds_request,
            "sound": self.handle_sound,
            "limits_request": self.handle_limits_request,
            "map_request": self.handle_map_request,
            "navigation": self.handle_navigation,
            "navigation_stop": self.handle_navigation_stop,
        }
        self.active_goal_handle = None

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        topics = self.settings.topics
        qos = qos_profile_sensor_data

        self.joystick_publisher = self.create_publisher(JoystickCommand, topics.joystick_publisher, qos)
        self.control_publisher = self.create_publisher(Control, topics.control_publisher, qos)
        self.sound_publisher = self.create_publisher(Sound, topics.sound_publisher, qos)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, topics.initial_pose_publisher, 10)
        self.navigate_client = ActionClient(self, NavigateToPose, topics.navigate_action)
        self.navigate_through_client = ActionClient(self, NavigateThroughPoses, topics.navigate_through_action)
        self.create_subscription(PoseWithCovarianceStamped, topics.robot_pose_subscriber, self.handle_robot_pose, 10)
        self.goal_status_state = {}
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action in (topics.navigate_action, topics.navigate_through_action):
            self.create_subscription(
                GoalStatusArray, action + "/_action/status", self.make_goal_status_callback(action), status_qos
            )
        self.topic_logger = TopicLogger(self, self.settings.logging)

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================
        self.create_timer(1.0 / self.settings.motion.publish_rate, self.publish_state)

    def set_browser_sender(self, sender):
        self.send_to_browser = sender

    def handle_browser_event(self, event):
        handler = self.browser_event_handlers.get(event.get("type"))
        if handler is not None:
            with self.teleop_lock:
                handler(event)

    def publish_state(self):
        with self.teleop_lock:
            teleop = self.teleop

            command = JoystickCommand()
            command.linear_velocity = teleop.linear_velocity
            command.angular_velocity = teleop.angular_velocity
            command.middle_mode = teleop.middle_mode
            command.middle_action = teleop.middle_action

            control = Control()
            control.mode = teleop.mode
            control.authority = teleop.authority

            teleop.clear_middle()

        self.joystick_publisher.publish(command)
        self.control_publisher.publish(control)

    def handle_joystick(self, event):
        limits = self.settings.motion
        self.teleop.set_velocity(
            event.get("linear", 0.0),
            event.get("angular", 0.0),
            limits.max_linear_velocity,
            limits.max_angular_velocity,
        )

    def handle_mode(self, event):
        self.teleop.set_mode(event.get("value", "manual"))

    def handle_authority(self, event):
        self.teleop.set_authority(event.get("value", ""))

    def handle_middle(self, event):
        self.teleop.set_middle(event.get("mode", ""), event.get("action", ""))

    def handle_sounds_request(self, event):
        if self.send_to_browser is not None:
            self.send_to_browser({"type": "sounds", "list": self.sound_names})

    def handle_limits_request(self, event):
        if self.send_to_browser is not None:
            limits = self.settings.motion
            self.send_to_browser({"type": "limits", "linear": limits.max_linear_velocity, "angular": limits.max_angular_velocity})

    def handle_sound(self, event):
        requested = event.get("play", "")
        if requested in self.sound_names:
            self.sound_publisher.publish(Sound(object=requested))

    def handle_map_request(self, event):
        if self.send_to_browser is None:
            return
        try:
            self.send_to_browser({"type": "map", **load_navigation_map()})
        except Exception:
            self.send_to_browser({"type": "map", "error": "map not available"})
            self.get_logger().warning(
                f"Could not load the navigation map in handle_map_request, "
                f"is the navigation package built?\n{traceback.format_exc()}"
            )

    def handle_navigation(self, event):
        if self.teleop.mode != "auto" or self.teleop.authority not in NAVIGATION_AUTHORITIES:
            self.get_logger().warning(
                "Navigation request rejected in handle_navigation, it needs auto mode with robot or shared authority"
            )
            return

        stamp = self.get_clock().now().to_msg()
        start = event.get("start")
        goal = event.get("goal")
        waypoints = event.get("waypoints") or []

        if start is not None:
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = stamp
            initial_pose.header.frame_id = "map"
            self.fill_pose(initial_pose.pose.pose, start)
            initial_pose.pose.covariance[0] = 0.25
            initial_pose.pose.covariance[7] = 0.25
            initial_pose.pose.covariance[35] = 0.068
            self.initial_pose_publisher.publish(initial_pose)

        if goal is None:
            return
        if waypoints:
            self.send_navigation_through(waypoints + [goal], stamp)
        else:
            self.send_navigation_goal(goal, stamp)

    def stamped_pose(self, values, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        self.fill_pose(pose.pose, values)
        return pose

    def send_navigation_goal(self, goal, stamp):
        if not self.navigate_client.server_is_ready():
            self.get_logger().warning(
                "Navigation goal dropped in send_navigation_goal, the Nav2 navigate_to_pose "
                "action server is not available, is the navigation launch running?"
            )
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.stamped_pose(goal, stamp)
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.store_goal_handle)

    def send_navigation_through(self, poses, stamp):
        if not self.navigate_through_client.server_is_ready():
            self.get_logger().warning(
                "Navigation goal dropped in send_navigation_through, the Nav2 navigate_through_poses "
                "action server is not available, is the navigation launch running?"
            )
            return
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self.stamped_pose(values, stamp) for values in poses]
        future = self.navigate_through_client.send_goal_async(goal_msg)
        future.add_done_callback(self.store_goal_handle)

    def store_goal_handle(self, future):
        try:
            handle = future.result()
        except Exception:
            self.get_logger().warning(f"Navigation goal was not accepted in store_goal_handle\n{traceback.format_exc()}")
            return
        self.active_goal_handle = handle if handle is not None and handle.accepted else None

    def handle_navigation_stop(self, event):
        handle = self.active_goal_handle
        self.active_goal_handle = None
        if handle is None:
            return
        handle.cancel_goal_async()
        self.get_logger().info("Navigation goal cancelled from the webpage stop button")

    def make_goal_status_callback(self, action):
        self.goal_status_state[action] = {"primed": False, "finished": set()}

        def callback(message):
            self.handle_goal_status(message, self.goal_status_state[action])

        return callback

    def handle_goal_status(self, message: GoalStatusArray, state):
        succeeded = [
            bytes(status.goal_info.goal_id.uuid)
            for status in message.status_list
            if status.status == GoalStatus.STATUS_SUCCEEDED
        ]
        if not state["primed"]:
            state["primed"] = True
            state["finished"].update(succeeded)
            return
        for goal_id in succeeded:
            if goal_id in state["finished"]:
                continue
            state["finished"].add(goal_id)
            clip = self.settings.sounds.goal_reached
            if clip:
                self.sound_publisher.publish(Sound(object=clip))
        if len(state["finished"]) > 64:
            state["finished"] = set(succeeded)

    def handle_robot_pose(self, message: PoseWithCovarianceStamped):
        if self.send_to_browser is None:
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.send_to_browser({"type": "robot_pose", "x": position.x, "y": position.y, "yaw": yaw})

    @staticmethod
    def fill_pose(pose, values: dict):
        pose.position.x = float(values.get("x", 0.0))
        pose.position.y = float(values.get("y", 0.0))
        yaw = float(values.get("yaw", 0.0))
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)


def run_web_server(node, pages_directory):
    try:
        start_web_server(node, pages_directory)
    except Exception:
        node.get_logger().error(
            f"Web server stopped, the browser interface is unreachable on "
            f"{node.settings.server.host}:{node.settings.server.port}\n{traceback.format_exc()}"
        )


def main():
    rclpy.init()
    node = WebInterfaceNode()
    pages_directory = find_pages_directory()

    server_thread = threading.Thread(target=run_web_server, args=(node, pages_directory), daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
