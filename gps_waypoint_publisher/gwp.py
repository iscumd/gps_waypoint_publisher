from cmath import cos, pi, sin
from math import atan2
from typing import List
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.action
import utm
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import NavSatFix, MagneticField


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

        self.wpp_handle = self.create_publisher(
            PoseArray, "/gps/points", qos_profile=rclpy.qos.qos_profile_system_default)

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

    def convert_gps(self) -> List[Pose]:
        with open(self.filepath) as f:
            out_points: List[Pose] = []

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

                ps = Pose()
                # Apply vector rotation
                ps.position.x = cos(
                    self.first_bearing).real * x - sin(self.first_bearing).real * y
                ps.position.y = sin(
                    self.first_bearing).real * x + cos(self.first_bearing).real * y

            self.get_logger().info("Converted points:")
            for (i, point) in enumerate(out_points):
                self.get_logger().info("point {}: x={}; y={};".format(
                    i, point.position.x, point.position.y))

            return out_points

    def start_waypoint_following(self, points: List[Pose]):
        pa = PoseArray()
        pa.poses = points
        pa.header.frame_id = "map"
        pa.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info("Sent points to wpp")
        self.wpp_handle.publish(pa)
