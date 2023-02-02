# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String, Int8

import cv2 # OpenCV library
import numpy as np
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image, CompressedImage # Image is the message type
import time
import math

 
 
class ReadCamera(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('read_camera_node')
            
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            '/cam/image_raw', 
            self.read_camera_callback, 
            10)
        self.subscription # prevent unused variable warning
            
        self.pub_line_sensor_raw = self.create_publisher(String, '/line_sensor/raw', 10)
        self.pub_line_sensor_val = self.create_publisher(Int8, '/line_sensor/val', 10)
        self.pub_line_sensor_img = self.create_publisher(Image, '/line_sensor/img', 10)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        
        self.fps = 0.0
        self.current_time=time.time()
        self.previous_time = time.time()


    # def scale_val(self, val):
    #     from_min, from_max = (-8.0, 8.0)
    #     from_range = from_max-from_min
        
    #     to_min, to_max = (-1.0, 1.0)
    #     to_range = to_max-to_min
        
    #     from_val = float(val)
    #     to_val = (((from_val-from_min)/from_range)*to_range)+to_min
        
    #     return to_val

    def reading_to_value(self, reading):
        val = None
        
        if reading == '100000000':
            val = -8
        elif reading == '110000000':
            val = -7
        elif reading == '010000000':
            val = -6
        elif reading == '011000000':
            val = -5
        elif reading == '001000000':
            val = -4
        elif reading == '001100000':
            val = -3
        elif reading == '000100000':
            val = -2
        elif reading == '000110000':
            val = -1
            
        elif reading == '000010000':
            val = 0
            
        elif reading == '000011000':
            val = 1
        elif reading == '000001000':
            val = 2
        elif reading == '000001100':
            val = 3
        elif reading == '000000100':
            val = 4
        elif reading == '000000110':
            val = 5
        elif reading == '000000010':
            val = 6
        elif reading == '000000011':
            val = 7
        elif reading == '000000001':
            val = 8
        
        
        elif reading == '000000000':
            val = -100
        else:
            val = 100
        
        return val

    

    def line_sense_read(self, img_bw, start_pnts, end_pnts):
        reading = [0,0,0,0,0,0,0,0,0]
        total_count = 15*15
        threshold = 45   # in percentage - measures the sensitivity in tracking lines
        
        for i in range(len(start_pnts)):
            y_start, y_end, x_start, x_end = start_pnts[i][1], end_pnts[i][1], start_pnts[i][0], end_pnts[i][0]
            box = img_bw[y_start:y_end, x_start:x_end]
            count=np.count_nonzero(box == 0)
            percent_count = count*100/total_count
            if percent_count>threshold:
                reading[i] = 1
            else:
                reading[i] = 0
                
        # using list comprehension
        readingToStr = ''.join([str(elem) for elem in reading])
        
        return readingToStr
    
    
    def line_sense(self, img_bw, img):
        # perform linesensing oeration
        start_pnts = [0,0,0,0,0,0,0,0,0]
        end_pnts = [0,0,0,0,0,0,0,0,0]
        
        color = (0,255,0) # BGR
        # thickness = cv2.FILLED # fill the rectangle with the color
        thickness = 2        
        
        img_width, img_height = img_bw.shape[1], img_bw.shape[0]
        no_of_box = 9
        no_of_side_box = int((no_of_box-1)/2)
        center_pos = int(math.floor(no_of_box/2))
        box_height = 15
        box_width = 15
        excess_space = img_width-(no_of_box*box_width)
        spacing = int(excess_space/(no_of_box+1))
        start_height = 75
        
        center_box_start_pnt = (int((img_width/2)-(box_width/2)), start_height)
        center_box_end_pnt = (center_box_start_pnt[0]+box_width, center_box_start_pnt[1]+box_height)
        start_pnts[center_pos] = center_box_start_pnt
        end_pnts[center_pos] = center_box_end_pnt
        cv2.rectangle(img, center_box_start_pnt,center_box_end_pnt, color, thickness)
        
        for n in range(no_of_side_box):
            lstart_pnt = (center_box_start_pnt[0]-((n+1)*(box_height+spacing)),start_height)
            lend_pnt = (lstart_pnt[0]+box_width, lstart_pnt[1]+box_height)
            start_pnts[no_of_side_box-(n+1)] = lstart_pnt
            end_pnts[no_of_side_box-(n+1)] = lend_pnt
            cv2.rectangle(img, lstart_pnt, lend_pnt, color, thickness)
            
            rstart_pnt = (center_box_start_pnt[0]+((n+1)*(box_height+spacing)),start_height)
            rend_pnt = (rstart_pnt[0]+box_width, rstart_pnt[1]+box_height)
            start_pnts[no_of_side_box+(n+1)] = rstart_pnt
            end_pnts[no_of_side_box+(n+1)] = rend_pnt
            cv2.rectangle(img, rstart_pnt, rend_pnt, color, thickness)
            
        reading = self.line_sense_read(img_bw, start_pnts, end_pnts)
        reading_val = self.reading_to_value(reading)
        
        return img, reading, reading_val


    def publish_readings(self, img, raw_reading, val):
        raw = String()
        raw.data = raw_reading
        
        value = Int8()
        value.data = int(val)
        
        self.pub_line_sensor_img.publish(self.br.cv2_to_imgmsg(img)) 
        self.pub_line_sensor_raw.publish(raw)
        self.pub_line_sensor_val.publish(value)



    def read_camera_callback(self, msg):
        self.current_time=time.time()
        """
        Callback function.
        """

        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(msg, "bgr8")  # current_frame = self.br.compressed_imgmsg_to_cv2(msg)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        threshold, img_bw = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)
        img, reading, val = self.line_sense(img_bw, img)
        
        self.publish_readings(img, reading, val)

        # update framerate
        self.fps = 1/(self.current_time-self.previous_time)
        self.previous_time=self.current_time
        
        
        # # Display image
        # cv2.imshow("camera", img)
        # # Display the message on the console
        # self.get_logger().info('readings=%s, val=%d, fps=%d' % (reading, int(val), int(self.fps)))

        cv2.waitKey(1)
  
    
  
  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    read_cam = ReadCamera()

    # Spin the node so the callback function is called.
    rclpy.spin(read_cam)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_cam.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
  

if __name__ == '__main__':
    main()