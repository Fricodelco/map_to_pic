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
        self.new_img = False
    def map_callback(self, map):
        # print(len(map.data))
        # map_img = np.zeros(())
        # print(map.info.resolution)
        # print(map.info.height)
        # print(map.info.width)
        blank_image = np.zeros((map.info.height, map.info.width,1), np.uint8)
        # blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
        for i in range(0, map.info.height):
            for j in range(0, map.info.width):
                k = j + (map.info.height - i - 1)*map.info.width
                if map.data[int(k)] == 0:
                    blank_image[i][j] = 254
                elif map.data[int(k)] == 100:
                    blank_image[i][j] = 0
                else:
                    blank_image[i][j] = 205
                
                # if map.data[i+j] != 100 and map.data[i+j] != 100:
                    # print(map.data[i+j])
        self.img = blank_image
        self.new_img = True
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('map_to_pic')
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)
    minimal_subscriber = MinimalSubscriber(node)

    while rclpy.ok():
        try:
            # print(type(minimal_subscriber.img))
            if minimal_subscriber.img is not None:
                # print(len(minimal_subscriber.img))
                if minimal_subscriber.new_img == True:
                    # print(minimal_subscriber.img)
                    img = minimal_subscriber.img
                    cv2.imwrite('Test_gray.jpg', img)
                    print(img.shape)
                    print(cv2.countNonZero(img))
                    scale_percent = 1000
                    width = int(img.shape[1] * scale_percent / 100)
                    height = int(img.shape[0] * scale_percent / 100)
                    dim = (width, height)
                    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
                    cv2.destroyAllWindows()
                    cv2.imshow('img',resized)
                    minimal_subscriber.new_img = False
                rate.sleep()
        except KeyboardInterrupt:
            rclpy.shutdown()
            node.destroy_node()
            thread.join()
    
    

if __name__ == '__main__':
    main()