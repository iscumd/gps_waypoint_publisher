from math import degrees, cos, sin
from operator import truediv
from typing import List, Optional, Tuple

import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.action
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener

from pygeodesy.ecef import EcefKarney, Ecef9Tuple
from pygeodesy.ellipsoids import Ellipsoids
from pygeodesy.utm import Utm, toUtm8

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose


class GpsWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')

        # Initalpose callback blocks, so we run the gps callback in its own thread
        par_cb = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timestamp from pose message
        self.stamp: Optional[Time] = None

        # The first GPS (lat, lon) point received after ip received
        self.first_gps: Optional[Tuple[float, float]] = None
        # The first bearing (in radians) received after ip received
        self.first_bearing: Optional[float] = None
        # Wether the initial pose has been received yet
        self.ip_received = False

        self.filepath: str = self.declare_parameter(
            'filepath', value=None).get_parameter_value().string_value
        if self.filepath is None:
            raise BaseException("filepath param must be set")

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initalpose, rclpy.qos.qos_profile_system_default, callback_group=par_cb)
        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.on_pose, rclpy.qos.qos_profile_sensor_data, callback_group=par_cb)

        self.wpp_handle = self.create_publisher(
            PoseArray, "/gps/points", qos_profile=rclpy.qos.qos_profile_system_default)

    def on_initalpose(self, msg: PoseWithCovarianceStamped):
        """ On initalpose publish, convert points and publish"""

        if msg.header.frame_id != "map":
            self.get_logger().error("Initial pose was not set in map frame!")
            return

        # This allows for us to receive pose
        self.ip_received = True

        self.get_logger().info("Waiting on a gps and mag lock...")

        # Wait until we get a lock.
        while self.first_gps is None or self.first_bearing is None:
            self.executor.spin_once(timeout_sec=1.0)
            self.get_logger().info("Still no gps lock...")

        self.get_logger().info("Lock obtained! Converting points...")

        points = self.convert_gps()
        self.start_waypoint_following(points)

    def on_pose(self, msg: PoseWithCovarianceStamped):
        if self.first_bearing is None and self.ip_received and msg.pose.pose.position.x != 0:
            self.get_logger().info("Got GPS pose!")
            
            # Save stamp for transform lookup later
            self.stamp = Time.from_msg(msg.header.stamp)

            (_, _, yaw) = euler_from_quaternion(
                [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
            self.first_bearing = 0  # yaw # In rad

            # convert from compass notation to our coordinates
            # if (-1* self.first_bearing) < 0:
            #    temp = 360.0 - self.first_bearing
            # else:
            #    temp = (-1*self.first_bearing)
            #self.first_bearing = temp

            self.get_logger().info(
                f"using a bearing of: {degrees(self.first_bearing)} degrees")

            # Convert the pose ECEF format to lat long
            converter = EcefKarney(Ellipsoids.GRS80)
            ecef_tup = (msg.pose.pose.position.x,
                        msg.pose.pose.position.y, msg.pose.pose.position.z)
            tup: Ecef9Tuple = converter.reverse(
                xyz=ecef_tup[0], y=ecef_tup[1], z=ecef_tup[2])  # Yes, this api is terrible
            (lat, long) = tup.latlon
            self.first_gps = (lat, long)

            self.get_logger().info(
                f"using a location of: lat: {lat}, long: {long}")

    def convert_gps(self) -> List[Pose]:
        """ Converts the gps points in file into poses relative to initalpose """

        with open(self.filepath) as f:
            out_points: List[Pose] = []

            # Convert the now lat,long point into UTM
            utm: Utm = toUtm8(latlon=self.first_gps[0], lon=self.first_gps[1])
            (first_utm_e, first_utm_n) = utm.eastingnorthing

            # Read lat, long from each line and convert
            for line in f.readlines():

                # Comment support
                if line.strip()[0] == "#":
                    continue

                split = line.strip().split(',')
                lat = float(split[0])
                lon = float(split[1])

                # Convert each point into UTM to remove spherical nonsense
                conv: Utm = toUtm8(latlon=lat, lon=lon)
                (easting, northing) = conv.eastingnorthing

                # UTM is in meteres, so we can just offset
                y = (easting - first_utm_e) * -1
                x = northing - first_utm_n

                ps = Pose()
                # Apply vector rotation
                ps.position.x = cos(
                    self.first_bearing) * x - sin(self.first_bearing) * y
                ps.position.y = sin(
                    self.first_bearing) * x + cos(self.first_bearing) * y

                self.get_logger().info(
                    f"Point before trans: {ps.position.x}, {ps.position.y}")

                try:
                    # Transform to map frame
                    trans = self.tf_buffer.lookup_transform(
                        'map', 'base_footprint', self.stamp, Duration(seconds=1))

                    ps.position.x += trans.transform.translation.x
                    ps.position.y += trans.transform.translation.y

                    facing_south = False #TODO change if facing a different direction

                    if facing_south: #TODO check
                        ps.position.x *= -1
                        ps.position.y *= -1

                    self.get_logger().info(f"Point after trans: {ps.position.x}, {ps.position.y}")
                    out_points.append(ps)
                except TransformException as ex:
                    self.get_logger().error(f"transform failed with: {ex}")

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
