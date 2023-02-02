# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


from threading import Thread
import time
from math import hypot



 
 
 
 
 
 
 
class FollowLine(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('follow_line_node')
        
        self.cmd_vel = Twist()
        self.pos = (0.0, 0.0) #x,y pose
        
        self.val = 0.0
        self.prev_reading = self.val
        self.prev_sensed_val = self.prev_reading
        
        self.sensor_setup = False
        

        self.action = 'n' # stores the current action to be taken
        self.prev_action = 'n'
        self.action_list = ['ff', 'f', 'l', 'r', 's']
        self.action_name = {
            'ff': 'following line',
            'f': 'moving forward',
            'l': 'turning left',
            'r': 'turning right',
            's': 'emergency stop',
            'n': 'stopped'
        }
        
        # self.e_stop = False
        
        
        self.subscription = self.create_subscription(
            Int8, 
            '/line_sensor/val', 
            self.read_val_callback, 
            10)
        self.subscription # prevent unused variable warning
        self.wait_count = 0
        
        self.subscription2 = self.create_subscription(
            Odometry, 
            '/odom', 
            self.read_odometry, 
            10)
        self.subscription2 # prevent unused variable warning
        self.wait_count2 = 0
        
        self.subscription3 = self.create_subscription(
            String, 
            '/line_follower_action/cmd', 
            self.get_action_cmd, 
            10)
        self.subscription3 # prevent unused variable warning
        
        
        self.publish_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        
        # create thread to handle the path tracking function without
        # interruption of the subscribed cam line sensor reading
        thread = Thread(target=self.exeute_actions)
        thread.daemon = True
        
        # thread2 = Thread(target=self.emergency_stop_check)
        # thread2.daemon = True
        
        thread.start()
        # thread2.start()
        

    








    ############# SIMPLE PATH TRACKING CONTROLLER #########################

    def map_w(self, val):
        val_min, val_max = (-8.0, 8.0)
        val_range = val_max-val_min
        
        w_min, w_max = (-0.9, 0.9)
        w_range = w_max-w_min
        
        w = (((float(val)-val_min)/val_range)*w_range)+w_min
        
        return -1*w
    
    def map_v(self, val):
        val_min, val_max = (-8, 0)
        val_range = val_max-val_min
        
        v_min, v_max = (0, 0.45)
        v_range = v_max-v_min
        
        new_val = -1*abs(val)
        v = (((float(new_val)-val_min)/val_range)*v_range)+v_min
        
        return v
    
    ###########################################################################
    
    
    
    
    
    
    ################# DIFFERNT POSSIBLE ACTIONS ###############################
    
    def track_line(self, val):
        self.cmd_vel.linear.x = self.map_v(int(val))
        self.cmd_vel.angular.z = self.map_w(int(val))
        self.publish_cmd.publish(self.cmd_vel)
    
    def stop(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.publish_cmd.publish(self.cmd_vel)
        
        
    def track_line_to_left(self):
        while True:
            if -6 < int(self.val) and  int(self.val) < 6:
                self.stop()
                break
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.6
            self.publish_cmd.publish(self.cmd_vel)
            
            time.sleep(0.04)
    
    def untrack_line_to_left(self):
        while True:
            if int(self.val) == -100:
                self.stop()
                break
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.6
            self.publish_cmd.publish(self.cmd_vel)
            
            time.sleep(0.04)
    
    def track_line_to_right(self):
        while True:
            if -6 < int(self.val) and  int(self.val) < 6:
                self.stop()
                break
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.6
            self.publish_cmd.publish(self.cmd_vel)
            
            time.sleep(0.04)
    
    def untrack_line_to_right(self):
        while True:
            if int(self.val) == -100:
                self.stop()
                break
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.6
            self.publish_cmd.publish(self.cmd_vel)
            
            time.sleep(0.04)
        
    
        
    def move_out_of_stop(self):
        while True:
            if int(self.val) == 100:
                self.cmd_vel.linear.x = 0.05
                self.cmd_vel.angular.z = 0.0
                self.publish_cmd.publish(self.cmd_vel)
            else:
                self.stop()
                break
            
            time.sleep(0.04)
            
            
                
    
    def add_clearance(self):
        x0,y0 = self.pos
        x1,y1 = self.pos
        d = hypot(x1-x0, y1-y0)
        c = 0.12
        
        while True:
            if d > c:
                self.stop()
                break
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.0
            self.publish_cmd.publish(self.cmd_vel)
            x1,y1 = self.pos
            d = hypot(x1-x0, y1-y0)
            # self.get_logger().info('val=%f' % (d))
            
            time.sleep(0.04)
            

    
    
    
    def follow_line(self):
        self.get_logger().info(self.action_name[self.action])
        while True:
            if int(self.val) == -100 or int(self.val) == 100:
                self.stop()
                if int(self.val) == -100:
                    if int(self.prev_sensed_val>0):
                        self.track_line_to_right()
                    else:
                        self.track_line_to_left() 
                else:
                    self.prev_action = self.action
                    self.action='n'
                    self.get_logger().info(self.action_name[self.action])
                    self.e_stop = False
                    break 
                
            else:
                self.track_line(self.val)
            
            time.sleep(0.04)

    
    def forward(self):
        self.get_logger().info(self.action_name[self.action])
        self.move_out_of_stop()
        self.prev_action = self.action
        self.action='n'
        self.get_logger().info(self.action_name[self.action])
    
    def turn_left(self):
        self.get_logger().info(self.action_name[self.action])
        if int(self.val)==100 or int(self.val)==-100:
            self.move_out_of_stop()
            self.add_clearance()
        elif self.prev_action=='f':
            self.add_clearance()
        self.untrack_line_to_left()
        self.track_line_to_left()
        self.prev_action = self.action
        self.action='n'
        self.get_logger().info(self.action_name[self.action])
        
    def turn_right(self):
        self.get_logger().info(self.action_name[self.action])
        if int(self.val)==100 or int(self.val)==-100:
            self.move_out_of_stop()
            self.add_clearance()
        elif self.prev_action=='f':
            self.add_clearance()
        self.untrack_line_to_right()
        self.track_line_to_right()
        self.prev_action = self.action
        self.action='n'
        self.get_logger().info(self.action_name[self.action])
        
    # def emergency_stop(self):
    #     self.get_logger().info(self.action_name[self.action])
    #     self.stop()
    #     self.prev_action = self.action
    #     # self.e_top = False
    #     self.action='n'
    #     self.get_logger().info(self.action_name[self.action])
    ##############################################################################





    
    
    ########### EXECUTING LINE FOLLOWING AND TRACKING ACTIONS ####################
    # def emergency_stop_check(self):
    #     while True:
    #         if self.action == 's':
    #             self.e_stop = True
    #             self.action = 'n'
    #         time.sleep(0.1)
            
            
    def exeute_actions(self):
        #### wait for sensors to properly setup #########
        self.get_logger().info('setting up sensors ....')
        while self.wait_count<20 and self.wait_count2<20:
            pass
        if not self.sensor_setup:
            self.get_logger().info('sensor setup finninshed !!!')
            self.sensor_setup = True
        ###################################################
        
        while True:
            if self.action in self.action_list:
                if self.action == 'ff':
                    self.follow_line()
                elif self.action == 'f':
                    self.forward()
                elif self.action == 'l':
                    self.turn_left()
                elif self.action == 'r':
                    self.turn_right()
            
            time.sleep(0.1)
        
    ################################################################################








    ################## READING, PRINTING, AND UPDATING SENSOR READING #################

    def print_readings(self, reading):
        if int(reading) != int(self.prev_reading):
            self.get_logger().info('val=%d' % (int(reading)))
            self.prev_sensed_val = self.prev_reading
            self.prev_reading = reading


    def read_val_callback(self, msg):
        if self.wait_count > 20: 
            self.val = float(msg.data)
            # self.print_readings(self.val)
        else:
            self.val = float(msg.data)
            # self.print_readings(self.val)
            self.wait_count += 1
        
    def read_odometry(self, msg):
        if self.wait_count2 > 20: 
            self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        else:
            self.wait_count2 += 1
            self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def get_action_cmd(self, msg):
        self.action = msg.data
        self.get_logger().info('%s action received' % (self.action))
        
    ####################################################################################    
  
    
  
  
  
  
  
  
  
  
  
  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    follow_line_node = FollowLine()

    # Spin the node so the callback function is called.
    rclpy.spin(follow_line_node)


    follow_line_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
  

if __name__ == '__main__':
    main()