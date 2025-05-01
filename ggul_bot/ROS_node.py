import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import json

def publish_detected_object(json_path):
    rclpy.init(args=None)
    node = Node('object_position_publisher')
    pub = node.create_publisher(Point, '/detected_object_position', 10)

    with open(json_path, 'r') as f:
        data = json.load(f)

    rate = node.create_rate(1)  # 1 Hz

    for obj in data['detected_objects']:
        point_msg = Point()
        point_msg.x = obj['X'] / 100.0  # cm → m
        point_msg.y = obj['Y'] / 100.0
        point_msg.z = obj['Z'] / 100.0
        node.get_logger().info(f"Publishing: {point_msg}")
        pub.publish(point_msg)
        rate.sleep()

    rclpy.shutdown()
