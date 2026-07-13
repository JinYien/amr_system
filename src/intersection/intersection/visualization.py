import math
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from intersection.detector import JUNCTION_LABELS
from intersection.geometry import normalize_angle, in_sector

OPEN_COLOR = ColorRGBA(r=0.15, g=0.85, b=0.25, a=1.0)
BLOCKED_COLOR = ColorRGBA(r=0.90, g=0.20, b=0.20, a=1.0)
ARROW_COLOR = ColorRGBA(r=0.10, g=0.95, b=0.95, a=1.0)
TEXT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
WEDGE_STEP = math.radians(5.0)


def _point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


class Visualizer:
    def __init__(self, node, topic, params):
        self.params = params
        self.publisher = node.create_publisher(MarkerArray, topic, 5)

    def has_subscribers(self) -> bool:
        return self.publisher.get_subscription_count() > 0

    def lifetime(self) -> Duration:
        seconds = int(self.params.marker_lifetime)
        nanoseconds = int((self.params.marker_lifetime - seconds) * 1e9)
        return Duration(sec=seconds, nanosec=nanoseconds)

    def base_marker(self, namespace, identifier, marker_type, frame, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = identifier
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = self.lifetime()
        marker.pose.orientation.w = 1.0
        return marker

    def publish(self, scan, params, detection, decision, junction):
        array = MarkerArray()
        stamp = scan.header.stamp
        frame = scan.header.frame_id

        sectors = (params.front, params.left, params.right)
        gaps = (detection.front, detection.left, detection.right)
        openings = (decision.front, decision.left, decision.right)

        for index, sector in enumerate(sectors):
            wedge = self.base_marker("sector_" + str(index), 0, Marker.TRIANGLE_LIST, frame, stamp)
            wedge.pose.position.z = -0.01
            wedge.scale.x = 1.0
            wedge.scale.y = 1.0
            wedge.scale.z = 1.0
            color = OPEN_COLOR if openings[index] else BLOCKED_COLOR
            wedge.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=float(self.params.sector_alpha))
            start = sector.min_angle - params.lidar_yaw_offset
            end = sector.max_angle - params.lidar_yaw_offset
            radius = params.min_open_distance
            segments = max(1, int(math.ceil(abs(end - start) / WEDGE_STEP)))
            for step in range(segments):
                first = start + (end - start) * step / segments
                second = start + (end - start) * (step + 1) / segments
                wedge.points.append(_point(0.0, 0.0))
                wedge.points.append(_point(radius * math.cos(first), radius * math.sin(first)))
                wedge.points.append(_point(radius * math.cos(second), radius * math.sin(second)))
            array.markers.append(wedge)

        points = self.base_marker("beams", 0, Marker.POINTS, frame, stamp)
        points.scale.x = self.params.point_size
        points.scale.y = self.params.point_size
        for index, range_value in enumerate(scan.ranges):
            if not math.isfinite(range_value) or range_value < scan.range_min or range_value > scan.range_max:
                continue
            raw = scan.angle_min + index * scan.angle_increment
            robot = normalize_angle(raw + params.lidar_yaw_offset)
            if not (in_sector(robot, params.front) or in_sector(robot, params.left) or in_sector(robot, params.right)):
                continue
            points.points.append(_point(range_value * math.cos(raw), range_value * math.sin(raw)))
            points.colors.append(OPEN_COLOR if range_value >= params.min_open_distance else BLOCKED_COLOR)
        array.markers.append(points)

        for index, gap in enumerate(gaps):
            arrow = self.base_marker("gap_direction", index, Marker.ARROW, frame, stamp)
            if not openings[index] or not gap.has_gap:
                arrow.action = Marker.DELETE
                array.markers.append(arrow)
                continue
            arrow.scale.x = 0.04
            arrow.scale.y = 0.10
            arrow.scale.z = 0.15
            arrow.color = ARROW_COLOR
            arrow.points = [
                _point(0.0, 0.0),
                _point(gap.distance * math.cos(gap.center_angle), gap.distance * math.sin(gap.center_angle)),
            ]
            array.markers.append(arrow)

        label = self.base_marker("junction_label", 0, Marker.TEXT_VIEW_FACING, frame, stamp)
        label.pose.position.z = 0.3
        label.scale.z = 0.2
        label.color = TEXT_COLOR
        label.text = JUNCTION_LABELS.get(junction, "unknown")
        array.markers.append(label)

        self.publisher.publish(array)
