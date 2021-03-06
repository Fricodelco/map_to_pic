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
class MapToPic(Node):
    def __init__(self, robot_radius):
        super().__init__('map_to_pic')
        self.subscription_1 = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.subscription_2 = self.create_subscription(TFMessage,'tf',self.tf_callback,10)
        self.subscription_3 = self.create_subscription(LaserScan,'scan_1',self.scan_callback,1)
        self.subscription_4 = self.create_subscription(OccupancyGrid,'keepout_filter_mask',self.keepout_callback,10)
        self.subscription_5 = self.create_subscription(Path,'plan',self.plan_callback,10)
        self.create_timer(0.5, self.timer_callback)
        # self.subscription  # prevent unused variable warning
        self.map_ = None
        self.keepout_map = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.map_resolution = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.robot_pose_img = [0,0,0,0]
        self.robot_radius = robot_radius
        self.tf_time = self.get_clock().now().to_msg()
        self.skip_tf = 0
        self.scan_buffer = [0]*100
        self.scan_angles = [0]*100
        self.robot_angle = 0
        self.plan = []
        
    def plan_callback(self, plan):
        try:
            self.plan = []
            for pose in plan.poses:
                x = (pose.pose.position.x + self.map_origin_x)/self.map_resolution
                y = (-pose.pose.position.y + self.map_origin_y)/self.map_resolution
                self.plan.append([x,y])
        
        except Exception as e:
            print(e)
    def scan_callback(self, data):
        increment = int(len(data.ranges)/len(self.scan_buffer))                
        j = 0
        for i in range(0, len(self.scan_buffer)*increment, increment):
            self.scan_buffer[j] = data.ranges[i]
            self.scan_angles[j] = i*data.angle_increment + data.angle_min
            j+=1
    def tf_callback(self, tf):
        # if self.skip_tf == 50:
            # self.tf_time = tf.transforms[0].header.stamp
            # self.tf_time.sec -= 1
            # self.skip_tf = 0
        # else:
            # self.skip_tf+=1
        if tf.transforms[0].child_frame_id == 'base_link':
            self.tf_time = tf.transforms[0].header.stamp
    def timer_callback(self):
        try:
            # print(self.get_clock().now().to_msg())
            # now = self.get_clock().now().to_msg()
            # now.nanosec -= 10000000
            # trans = self.tfBuffer.lookup_transform('map', 'base_link', now)
            trans = self.tfBuffer.lookup_transform('map', 'base_link', self.tf_time)
            translation = trans.transform.translation 
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_angle = atan2(siny_cosp, cosy_cosp)+pi/2

            # # angle = asin(2*rotation.x*rotation.y+2*rotation.z*rotation.w)
            # print(self.map_origin_x, self.map_origin_y)
            # print(translation.x, translation.y, "trans")
            if self.map_ is not None:
                delta_x = translation.x + self.map_origin_x
                delta_y = -translation.y + self.map_origin_y
                # print(delta_x, delta_y)
                self.robot_pose_img = [delta_x/self.map_resolution,
                delta_y/self.map_resolution, 
                delta_x/self.map_resolution + sin(self.robot_angle)*self.robot_radius/self.map_resolution*1.5,
                delta_y/self.map_resolution + cos(self.robot_angle)*self.robot_radius/self.map_resolution*1.5]
        except Exception as e:
            print(e)
    def map_callback(self, map):
        blank_image = np.zeros((map.info.height, map.info.width,3), np.uint8)
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                k = j + (map.info.height - i - 1)*map.info.width
                if map.data[int(k)] == 0:
                    blank_image[i][j][0] = 254
                    blank_image[i][j][1] = 254
                    blank_image[i][j][2] = 254
                elif map.data[int(k)] == 100:
                    blank_image[i][j][0] = 0
                    blank_image[i][j][1] = 0
                    blank_image[i][j][2] = 0
                else:
                    blank_image[i][j][0] = 200
                    blank_image[i][j][1] = 200
                    blank_image[i][j][2] = 200
        # self.map_origin = map.info.origin
        self.map_origin_x = -map.info.origin.position.x # tf map in glonal map px
        self.map_origin_y = map.info.height*map.info.resolution+map.info.origin.position.y
        self.map_resolution = map.info.resolution
        self.map_ = blank_image
    def keepout_callback(self, map):
        blank_image = np.zeros((map.info.height, map.info.width,3), np.uint8)
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                k = j + (map.info.height - i - 1)*map.info.width
                if map.data[int(k)] == 0:
                    blank_image[i][j][0] = 254
                    blank_image[i][j][1] = 254
                    blank_image[i][j][2] = 254
                elif map.data[int(k)] == 100:
                    blank_image[i][j][0] = 0
                    blank_image[i][j][1] = 0
                    blank_image[i][j][2] = 0
                else:
                    blank_image[i][j][0] = 200
                    blank_image[i][j][1] = 200
                    blank_image[i][j][2] = 200
        # self.map_origin = map.info.origin
        self.map_origin_x = -map.info.origin.position.x # tf map in glonal map px
        self.map_origin_y = map.info.height*map.info.resolution+map.info.origin.position.y
        self.map_resolution = map.info.resolution
        self.keepout_map = blank_image
    def get_map(self):
        image_ = None
        if self.keepout_map is not None:
            image_ = copy(self.keepout_map)
        elif self.map_ is not None:
            image_ = copy(self.map_)
        if image_ is not None:
            scale = 5
            width = int(image_.shape[1] * scale)
            height = int(image_.shape[0] * scale)
            dim = (width, height)
            image_ = cv2.resize(image_, dim, interpolation = cv2.INTER_AREA)
            for i in range(0, len(self.scan_buffer)-1,1):
                if isnan(self.scan_buffer[i]) == False:
                    # print(self.scan_buffer[i])
                    position_x = self.robot_pose_img[0]+sin(self.robot_angle+self.scan_angles[i])*self.scan_buffer[i]/self.map_resolution
                    position_y = self.robot_pose_img[1]+cos(self.robot_angle+self.scan_angles[i])*self.scan_buffer[i]/self.map_resolution
                    cv2.circle(image_, (int(position_x)*scale, int(position_y)*scale), 1, (0, 0, 255), 1)
            for pose in self.plan:
                cv2.circle(image_, (int(pose[0]*scale), int(pose[1]*scale)), 1, (255,0,0),1)
            cv2.circle(image_, (int(self.robot_pose_img[0])*scale, int(self.robot_pose_img[1])*scale), int(self.robot_radius/self.map_resolution)*scale, (255, 0, 0), 1)
            cv2.line(image_, (int(self.robot_pose_img[0])*scale, int(self.robot_pose_img[1])*scale),
            (int(self.robot_pose_img[2])*scale, int(self.robot_pose_img[3])*scale),(255,0,0),1)
            # msg = Image()
            # msg = self.bridge.cv2_to_imgmsg(image_, "bgr8")
            # self.publisher_.publish(msg)
            return image_
        else:
            return False
    
    def get_map_origin(self):
        return self.map_origin_x, self.map_origin_y
    
    def get_map_resolution(self):
        return self.map_resolution

def main(args=None):
    rclpy.init(args=args)
    map_to_pic = MapToPic(0.3)
    thread = threading.Thread(target=rclpy.spin, args=(map_to_pic, ), daemon=True)
    thread.start()
    rate = map_to_pic.create_rate(2)
    while rclpy.ok():
        try:
            # print(type(minimal_subscriber.img))
            if map_to_pic.map_ is not None:
                # print(len(minimal_subscriber.img))
                img = map_to_pic.get_map()
                # img = map_to_pic.map_
                # cv2.imwrite('Test_gray.jpg', img)
                # scale_percent = 400
                # width = int(img.shape[1] * scale_percent / 100)
                # height = int(img.shape[0] * scale_percent / 100)
                # dim = (width, height)
                # resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
                # cv2.imshow('img',img)
                # cv2.waitKey(1)
                rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()
            map_to_pic.destroy_node()
            thread.join()
            cv2.destroyAllWindows()
    
    

if __name__ == '__main__':
    main()