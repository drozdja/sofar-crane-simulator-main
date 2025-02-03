import arcade
import rclpy
import time
from enum import Enum
from threading import Thread
from rclpy.node import Node
from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Point
from simple_pid import PID


#Robot Logic Node
class RobotLogicNode(Node):
    def __init__(self):
        super().__init__('robot_logic_node')
        self.current_stage = 1
        self.x_ack = False
        self.y_ack = False

        #Subscribe to ack_x and ack_y
        self.ack_x_sub = self.create_subscription(Bool, 'ack_x', self.ack_x_callback, 10)
        self.ack_y_sub = self.create_subscription(Bool, 'ack_y', self.ack_y_callback, 10)
        #Publish next stage
        self.next_stage_pub = self.create_publisher(Int64, 'next_stage', 10)
        time.sleep(5)
        self.ack_x_received = False
        self.ack_y_received = False

    def ack_x_callback(self, msg: Bool):
        self.x_ack = msg.data

    def ack_y_callback(self, msg: Bool):
        self.y_ack = msg.data

    def publish_next_stage(self):
        if self.x_ack and self.y_ack:
            next_stage_msg = Int64()
            next_stage_msg.data = self.current_stage
            self.next_stage_pub.publish(next_stage_msg)
            self.current_stage += 1
            #reset if all the stages are completed
            if self.current_stage > 5:
                self.current_stage = 1
            self.x_ack = False
            self.y_ack = False

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