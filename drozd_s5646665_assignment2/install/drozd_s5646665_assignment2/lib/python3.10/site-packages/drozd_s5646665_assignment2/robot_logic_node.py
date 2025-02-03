import arcade
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Point
from simple_pid import PID

# Robot Logic Node
class RobotLogicNode(Node):
    def __init__(self):
        super().__init__('robot_logic_node')
        # Subscribe to ack_x and ack_y
        self.ack_x_sub = self.create_subscription(Bool, 'ack_x', self.ack_x_callback, 10)
        self.ack_y_sub = self.create_subscription(Bool, 'ack_y', self.ack_y_callback, 10)
        # Publish next stage
        self.next_stage_pub = self.create_publisher(Int64, 'next_stage', 10)

        self.ack_x_received = False
        self.ack_y_received = False

    def ack_x_callback(self, msg):
        if msg.data:
            self.ack_x_received = True
            self.check_acknowledgements()

    def ack_y_callback(self, msg):
        if msg.data:
            self.ack_y_received = True
            self.check_acknowledgements()

    def check_acknowledgements(self):
        if self.ack_x_received and self.ack_y_received:
            self.ack_x_received = False
            self.ack_y_received = False
            self.next_stage_pub.publish(Int64(data=1))

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation and spin indefinitely..
    robot_logic = RobotLogicNode()
    rclpy.spin(robot_logic)

     # On shutdown...
    robot_logic.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()