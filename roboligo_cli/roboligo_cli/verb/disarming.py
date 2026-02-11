import rclpy
from ros2cli.node.direct import DirectNode

from roboligo_interfaces.srv import RoboligoString

from ros2cli.verb import VerbExtension


class DisarmingVerb(VerbExtension):
    """roboligo disarming command."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
                  '--value', '-v', action='store_true',
                  help="Disarming the robot, just work for Drones and Rovers.")

    def main(self, *, args):
        with DirectNode(args) as node:
            client = node.create_client(RoboligoString, '/roboligo/input')
            while not client.wait_for_service(timeout_sec=2.0):
                node.get_logger().info('Service not available, try again')
            req = RoboligoString.Request()
            req.data = 'disarming'
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future)