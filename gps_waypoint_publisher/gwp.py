from math import degrees, cos, sin
from typing import List, Optional, Tuple
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.action
from tf_transformations import euler_from_quaternion
from pygeodesy.ecef import EcefKarney, Ecef9Tuple
from pygeodesy.ellipsoids import Ellipsoids
from pygeodesy.utm import Utm, toUtm8
from pygeodesy.points import LatLon_
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


class GpsWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')

        # The first GPS point received after ip received
        self.first_gps: Optional[Tuple[float, float]] = None
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
        # A magnometer reading, for bearing
        self.create_subscription(
            PoseStamped, "/pose", self.on_pose, rclpy.qos.qos_profile_sensor_data)

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

    def on_pose(self, msg: PoseStamped):
        if self.first_bearing is None and self.ip_received:
            self.get_logger().info("Got Orientation!")

            (_, _, _, yaw) = euler_from_quaternion(msg.pose.orientation)
            self.first_bearing = yaw

            self.get_logger().info(f"using a bearing of: {degrees(self.first_bearing)} degrees")

            # Convert the pose ECEF format to lat long
            converter = EcefKarney(Ellipsoids.GRS80)
            tup: Ecef9Tuple = converter.reverse(msg.pose.position)
            (lat, long) = tup.latlon
            self.first_gps = (lat, long)

    def convert_gps(self) -> List[Pose]:
        with open(self.filepath) as f:
            out_points: List[Pose] = []

            # Convert the now lat,long point into UTM
            utm: Utm = toUtm8(LatLon_(
                self.first_gps[0], self.first_gps[1]))
            (first_utm_e, first_utm_n) = utm.eastingnorthing

            # Read lat, long from each line and convert
            for line in f.readlines():

                # Comment support
                if line.strip()[0] == "#":
                    continue

                split = line.strip().split(',')
                lat = float(split[0])
                lon = float(split[1])

                conv: Utm = toUtm8(LatLon_(lat, lon))
                (easting, northing) = conv.eastingnorthing

                # UTM is in meteres, so we can just offset
                x = easting - first_utm_e
                y = northing - first_utm_n

                ps = Pose()
                # Apply vector rotation
                ps.position.x = cos(
                    self.first_bearing) * x - sin(self.first_bearing) * y
                ps.position.y = sin(
                    self.first_bearing) * x + cos(self.first_bearing) * y
                out_points.append(ps)

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
