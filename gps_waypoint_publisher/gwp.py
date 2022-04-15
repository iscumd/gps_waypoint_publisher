from cmath import cos, pi, sin
from math import atan2
from typing import List
import rclpy
import rclpy.logging
import rclpy.qos
import utm
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
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
            NavSatFix, "/gps", self.on_gps, rclpy.qos.qos_profile_system_default)
        # A magnometer reading, for bearing
        self.create_subscription(
            MagneticField, "/mag", self.on_mag, rclpy.qos.qos_profile_system_default)

    def on_initalpose(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != "map":
            self.get_logger().error("Initial pose was not set in map frame!")
            return

        # This allows for us to receive initial GPS and mag messages
        self.ip_received = True

        # Wait until we get a lock.
        while self.first_gps is None or self.first_bearing is None:
            rclpy.spin_once()

        self.convert_gps()
        # TODO send points

    def on_gps(self, msg: NavSatFix):
        if self.first_gps is not None and msg.latitude != 0 and self.ip_received:
            self.first_gps = msg

    def on_mag(self, msg: MagneticField):
        if self.first_bearing is not None and self.ip_received:
            # Convert magnetic fields to degrees from magnetic north (0 degrees). Ex. east is +90
            self.first_bearing = atan2(
                msg.magnetic_field.y, msg.magnetic_field.x) * 180 / pi

    def convert_gps(self) -> List[PoseStamped]:
        with open(self.filepath) as f:
            out_points: List[PoseStamped] = []

            (first_utm_e, first_utm_n, _, _) = utm.from_latlon(
                self.first_gps.latitude, self.first_gps.longitude)

            # Read lat, long from each line and convert
            for line in f.readlines():
                split = line.split(',')
                lat = float(split[0])
                lon = float(split[1])

                (easting, northing, _, _) = utm.from_latlon(lat, lon)

                # UTM is in meteres, so we can just offset
                x = easting - first_utm_e
                y = northing - first_utm_n

                ps = PoseStamped()
                # Apply vector rotation
                ps.pose.position.x = cos(
                    self.first_bearing) * x - sin(self.first_bearing) * y
                ps.pose.position.y = sin(
                    self.first_bearing) * x + cos(self.first_bearing) * y

                ps.header.frame_id = "map"
                ps.header.stamp = self.get_clock().now()

            return out_points
