import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time

class CommandRobot(Node):

    def __init__(self):
        super().__init__('command_robot_node')
        self.command = self.create_publisher(String, '/line_follower_action/cmd', 10)
        self.action_list = ('ff', 'f', 'l', 'r', 's')
        
        while rclpy.ok():
            action = input('enter command: ')
            if action in self.action_list:
                cmd = String()
                cmd.data = action
                self.command.publish(cmd)
                self.get_logger().info('%s action sent' % (action))
            # elif action == '':
            #     self.get_logger().info('shutting down ....')
            #     time.time(2.0)
            #     rclpy.shutdown()
            else:
                self.get_logger().info('invalid action comand')


def main(args=None):
    rclpy.init(args=args)

    command_node = CommandRobot()

    rclpy.spin(command_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()