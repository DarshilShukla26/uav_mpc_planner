import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from uav_mpc.planning.path_planner import PathPlanner # placeholder

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        self.path_pub = self.create_publisher(Path, '/uav/reference_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/uav/obstacles', 10)
        
        self.timer = self.create_timer(1.0, self.publish_references)
        self.get_logger().info("Visualizer node ready.")

    def publish_references(self):
        # Generate a dummy reference path moving forward
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(i * 0.1)
            pose.pose.position.z = 1.0 # Hover at 1m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
