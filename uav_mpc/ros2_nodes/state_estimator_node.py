import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class StateEstimatorNode(Node):
    """ Passes Gazebo ground truth or handles noisy EKF state estimation. """
    def __init__(self):
        super().__init__('state_estimator_node')
        
        # Subscribe to raw sensors/gt
        self.gt_sub = self.create_subscription(Odometry, '/gazebo/ground_truth', self.gt_callback, 10)
        
        # Publish estimated state to MPC
        self.odom_pub = self.create_publisher(Odometry, '/uav/odom', 10)
        
        self.get_logger().info("State Estimator Node ready.")

    def gt_callback(self, msg: Odometry):
        # Simplest form: pass ground truth directly. Real applications run EKF here.
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
