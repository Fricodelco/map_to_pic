import rclpy
from rclpy.node import Node
import subprocess

echo = subprocess.check_call(['sh', '/home/rodion/rodion_nav/src/map_to_pic/map_to_pic/nav_launch.sh'])
print(echo)