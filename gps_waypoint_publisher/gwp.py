from cmath import cos, pi, sin
from math import atan2
from typing import List
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.action
import utm
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import NavSatFix, MagneticField
from nav2_msgs.action import FollowWaypoints


class GpsWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')

        # The first GPS point received after ip received
        self.first_gps = None
        # The first bearing (in degrees) received after ip received
        self.first_bearing = None
        # Wether the initial pose has been received yet
        self.ip_received = False

        self.filepath: str = self.declare_parameter(
            'filepath', value=None).get_parameter_value().string_value
        if self.filepath is None:
            raise BaseException("filepath param must be set")

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initalpose, rclpy.qos.qos_profile_system_default)
        self.create_subscription(
            NavSatFix, "/gps", self.on_gps, rclpy.qos.qos_profile_sensor_data)
        # A magnometer reading, for bearing
        self.create_subscription(
            MagneticField, "/mag", self.on_mag, rclpy.qos.qos_profile_sensor_data)

        self.nav_client = rclpy.action.ActionClient(
            self, FollowWaypoints, "/follow_waypoints")

    def on_initalpose(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != "map":
            self.get_logger().error("Initial pose was not set in map frame!")
            return

        # This allows for us to receive initial GPS and mag messages
        self.ip_received = True

        self.get_logger().info("Waiting on a gps and mag lock...")

        # Wait until we get a lock.
        while self.first_gps is None or self.first_bearing is None:
            rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info("Still no gps lock...")

        self.get_logger().info("Lock obtained! Converting points...")

        points = self.convert_gps()
        self.start_waypoint_following(points)

    def on_gps(self, msg: NavSatFix):
        if self.first_gps is None and msg.latitude != 0:
            self.get_logger().info("Got GPS!")
            self.first_gps = msg

    def on_mag(self, msg: MagneticField):
        if self.first_bearing is None:
            self.get_logger().info("Got Mag!")
            # Convert magnetic fields to degrees from magnetic north (0 degrees). Ex. east is +90
            self.first_bearing = atan2(
                msg.magnetic_field.y, msg.magnetic_field.x) * 180 / pi
            self.get_logger().info("using a bearing of: {}".format(self.first_bearing))

    def convert_gps(self) -> List[PoseStamped]:
        with open(self.filepath) as f:
            out_points: List[PoseStamped] = []

            (first_utm_e, first_utm_n, _, _) = utm.from_latlon(
                self.first_gps.latitude, self.first_gps.longitude)

            # Read lat, long from each line and convert
            for line in f.readlines():

                # Comment support
                if line.strip()[0] == "#":
                    continue

                split = line.strip().split(',')
                lat = float(split[0])
                lon = float(split[1])

                (easting, northing, _, _) = utm.from_latlon(lat, lon)

                # UTM is in meteres, so we can just offset
                x = easting - first_utm_e
                y = northing - first_utm_n

                ps = PoseStamped()
                # Apply vector rotation
                ps.pose.position.x = cos(
                    self.first_bearing).real * x - sin(self.first_bearing).real * y
                ps.pose.position.y = sin(
                    self.first_bearing).real * x + cos(self.first_bearing).real * y

                ps.header.frame_id = "map"
                ps.header.stamp = self.get_clock().now().to_msg()
                out_points.append(ps)

            return out_points

    def start_waypoint_following(self, points: List[PoseStamped]):
        if not self.nav_client.wait_for_server(5):
            self.get_logger().error("FollowWaypoints action server is not available!")
        else:
            self.get_logger().info("Begginging to navigate to {} waypoints:".format(len(points)))

            for (i, point) in enumerate(points):
                self.get_logger().info("Point {}: x={};y={};".format(i,
                                       point.pose.position.x, point.pose.position.y))

            goal = FollowWaypoints.Goal()
            goal.poses = points
            fut = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)

            self.get_logger().info("Finished navigation.")
