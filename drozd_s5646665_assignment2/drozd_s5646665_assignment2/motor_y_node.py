import arcade
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from .lib.crane_sim import CraneSimulation

from std_msgs.msg import Float64, Int64, Bool
from geometry_msgs.msg import Point
from simple_pid import PID

#Motor Y Node
class MotorYNode(Node):
    def __init__(self):
        super().__init__('motor_y_node')
        #Subscribe to controller_setpoint topic to receive the target position
        self.setpoint_sub = self.create_subscription(
            Point, 'controller_setpoint', self.setpoint_callback, 10)
        #Publish to the topic
        self.motor_y_pub = self.create_publisher(Float64, 'motor_y', 10)
        self.ack_y_pub = self.create_publisher(Bool, 'ack_y', 10)

        #Init PID
        self.pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0)
        self.pid.output_limits = (-1, 1)
        self.current_y = 0

    def setpoint_callback(self, msg):
        self.pid.setpoint = msg.y
        reached_target = False
        threshold = 0.01
        #infinite loop until the error is below threshold
        while not reached_target:
            #convert velocity to postion
            delta_y = self.pid(self.current_y)
            self.current_y += delta_y
            self.pub_motor_y.publish(Float64(data=self.current_y))

            if abs(self.current_y - self.pid.setpoint) < threshold:
                reached_target = True
                self.pub_ack_y.publish(Bool(data=True))

# Main function
def main(args=None):
    # Create node
    rclpy.init(args=args)
    motor_y_node = MotorYNode()
    rclpy.spin(motor_y_node)

    # On shutdown...
    motor_y_node.destroy_node()
    rclpy.shutdown()


# Script entry point
if __name__ == '__main__':
    main()
