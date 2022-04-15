import rclpy
from .gwp import GpsWaypointPublisher

def main(args=None):
    rclpy.init(args=args)

    gwp = GpsWaypointPublisher()

    rclpy.spin(gwp)

    gwp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()