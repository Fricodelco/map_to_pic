import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import tf2_ros
from tf2_msgs.msg import TFMessage
from copy import copy
from math import asin, sin, cos, pi, atan2, isnan
from sensor_msgs.msg import LaserScan
import subprocess
from nav_msgs.msg import Path
from rclpy.qos import ReliabilityPolicy, QoSProfile

class MapToPic(Node):
    def __init__(self, robot_radius):
        super().__init__('map_to_pic')
        
        self.subscription_ = self.create_subscription(LaserScan,'scan',self.sc_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        
    
    def sc_callback(self, data):
        print("get scan")
     
  

def main(args=None):
    rclpy.init(args=args)
    conn = MapToPic(2)
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()
            
    

if __name__ == '__main__':
    main()
