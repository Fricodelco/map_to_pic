import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import tf2_ros
from tf2_msgs.msg import TFMessage
class MapToPic(Node):
    def __init__(self):
        super().__init__('map_to_pic')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.subscription = self.create_subscription(TFMessage,'tf_static',self.tf_callback,10)
        self.subscription  # prevent unused variable warning
        self.map = None
        self.new_img = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.map_resolution = 0
        self.map_origin = None
        self.robot_pose_img = [0,0,0]
    def tf_callback(self, tf):
        try:
            # trans = self.tfBuffer.lookup_transform('map', 'base_footprint', self.get_clock().now().to_msg())
            trans = self.tfBuffer.lookup_transform('map', 'base_footprint', tf.transforms[0].header.stamp)
            translation = trans.transform.translation 
            rotation = trans.transform.rotation
            if self.map is not None:
                delta_x = translation.x - self.map_origin.position.x
                delta_y = translation.y - self.map_origin.position.y
                # print(delta_x, delta_y)
                self.robot_pose_img = [delta_x/self.map_resolution,
                delta_y/self.map_resolution, 0]
                print(self.robot_pose_img)
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
        self.map_origin = map.info.origin
        # print(self.map_origin)
        self.map_resolution = map.info.resolution
        self.map = blank_image
        self.new_img = True
    def get_image(self):
        a=1
def main(args=None):
    rclpy.init(args=args)
    map_to_pic = MapToPic()
    thread = threading.Thread(target=rclpy.spin, args=(map_to_pic, ), daemon=True)
    thread.start()
    rate = map_to_pic.create_rate(10)
    while rclpy.ok():
        try:
            # print(type(minimal_subscriber.img))
            if map_to_pic.map is not None:
                # print(len(minimal_subscriber.img))
                # if minimal_subscriber.new_img == True:
                #     # print(minimal_subscriber.img)
                #     img = minimal_subscriber.img
                #     cv2.imwrite('Test_gray.jpg', img)
                #     print(img.shape)
                #     print(cv2.countNonZero(img))
                #     scale_percent = 1000
                #     width = int(img.shape[1] * scale_percent / 100)
                #     height = int(img.shape[0] * scale_percent / 100)
                #     dim = (width, height)
                #     resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
                #     cv2.destroyAllWindows()
                #     cv2.imshow('img',resized)
                #     minimal_subscriber.new_img = False
                rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()
            map_to_pic.destroy_node()
            thread.join()
    
    

if __name__ == '__main__':
    main()