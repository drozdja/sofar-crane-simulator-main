import arcade
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Point
from simple_pid import PID

#Motor X Node
class MotorXNode(Node):
    def __init__(self):
        super().__init__('motor_x_node')
        #Subscribe to the controller_setpoint topic to receive the target position
        self.setpoint_sub = self.create_subscription(
            Point, 'controller_setpoint', self.setpoint_callback, 10)
        #Publish to the topics
        self.motor_x_pub = self.create_publisher(Float64, 'motor_x', 10)
        self.ack_x_pub = self.create_publisher(Bool, 'ack_x', 10)

        #Init PID
        self.pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0)
        self.pid.output_limits = (-1, 1)
        self.current_x = 0

    def setpoint_callback(self, msg):
        self.pid.setpoint = msg.x
        reached_target = False
        threshold = 0.01
        #infinite loop until the error is below threshold
        while not reached_target:
            #convert velocity to postion
            delta_x = self.pid(self.current_x)
            self.current_x += delta_x
            self.pub_motor_x.publish(Float64(data=self.current_x))

            if abs(self.current_x - self.pid.setpoint) < threshold:
                reached_target = True
                self.pub_ack_x.publish(Bool(data=True))

# Main function
def main(args=None):
    # Create node
    rclpy.init(args=args)
    motor_x_node = MotorXNode()
    rclpy.spin(motor_x_node)

    # On shutdown...
    motor_x_node.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()
