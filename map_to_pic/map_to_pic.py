import rclpy
from rclpy.node import Node
import threading
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
class MinimalSubscriber():
    def __init__(self, node):
        self.subscription = node.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.img = None
    def map_callback(self, map):
        print(len(map.data))
        # map_img = np.zeros(())
        print(map.info.resolution)
        print(map.info.height)
        print(map.info.width)
        blank_image = np.zeros((map.info.height, map.info.width,1), np.uint8)
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                blank_image[i][j] = map.data[i+j]
        self.img = blank_image
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('map_to_pic')
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)
    minimal_subscriber = MinimalSubscriber(node)
    while rclpy.ok():
        try:
            if minimal_subscriber.img != None:
                print(len(minimal_subscriber.img))
                rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()
            node.destroy_node()
            thread.join()
    
    

if __name__ == '__main__':
    main()