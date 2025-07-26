import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

"""A ROS 2 node that publishes a wall marker for visualization in RViz.
    This marker represents a wall in front of the UR10e robot, useful for testing
    the robot's reach and interaction with the environment.
    The wall is defined as a cube with specific dimensions and position.
    The marker is published at a regular interval to ensure it is visible in RViz.
    The wall is positioned at (1.3, 0.0, 1.25) with a size of (0.1, 3.0, 2.5
    The color of the wall is set to a grayish tone (0.5, 0.5, 0.5) with full opacity (1.0).
    The marker is published on the 'visualization_marker' topic.
    The node is designed to run continuously, publishing the marker at a fixed rate.
    To run this node, ensure you have the necessary ROS 2 environment set up and
    execute the node using the command:
    ros2 run my_ur10e_custom_description wall_marker_publisher
    This will start the node and begin publishing the wall marker.
    Make sure to have RViz running and subscribe to the 'visualization_marker' topic
    to visualize the wall marker in the simulation environment."""

class WallMarkerPublisher(Node):
    def __init__(self):
        super().__init__('wall_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wall"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.3  # in front of robot
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5 / 2.0
        marker.scale.x = 0.1
        marker.scale.y = 3.0
        marker.scale.z = 2.5
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        self.publisher.publish(marker)

def main():
    rclpy.init()
    node = WallMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
