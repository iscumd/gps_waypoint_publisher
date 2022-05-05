import rclpy
import rclpy.executors
from .gwp import GpsWaypointPublisher

def main(args=None):
    rclpy.init(args=args)
    exe = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    gwp = GpsWaypointPublisher()

    exe.add_node(gwp)

    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gwp.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()