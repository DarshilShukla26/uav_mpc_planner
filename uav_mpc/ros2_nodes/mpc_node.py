import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Wrench, PoseStamped
import numpy as np
import time

from uav_mpc.controllers.mpc_controller import MPCController
from uav_mpc.controllers.receding_horizon import RecedingHorizonController

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        
        # ROS 2 Parameters
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('horizon', 20)
        
        freq = self.get_parameter('frequency').value
        N = self.get_parameter('horizon').value
        dt = 1.0 / freq
        
        # Initialize MPC
        self.get_logger().info(f"Initializing MPC with N={N}, dt={dt}")
        self.mpc = MPCController(N=N, dt=dt)
        self.controller = RecedingHorizonController(self.mpc)
        
        # State variables
        self.current_state = np.zeros(12)
        self.reference_trajectory = np.zeros((12, N + 1))
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/uav/reference_path', self.path_callback, 10)
        
        # Publishers
        self.control_pub = self.create_publisher(Wrench, '/uav/control_cmd', 10)
        
        # Timer for 50Hz loop
        self.timer = self.create_timer(dt, self.control_loop)
        
        self.get_logger().info("MPC Node ready.")

    def odom_callback(self, msg: Odometry):
        # Extract state from Odometry
        # Pos
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = msg.pose.pose.position.z
        
        # Vel
        self.current_state[3] = msg.twist.twist.linear.x
        self.current_state[4] = msg.twist.twist.linear.y
        self.current_state[5] = msg.twist.twist.linear.z
        
        # Note: We represent attitude as Euler angles. Need quaternion to Euler conversion.
        import math
        q = msg.pose.pose.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.current_state[6] = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.current_state[7] = math.copysign(math.pi / 2, sinp)
        else:
            self.current_state[7] = math.asin(sinp)
            
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_state[8] = math.atan2(siny_cosp, cosy_cosp)
        
        # Angular rates
        self.current_state[9] = msg.twist.twist.angular.x
        self.current_state[10] = msg.twist.twist.angular.y
        self.current_state[11] = msg.twist.twist.angular.z

    def path_callback(self, msg: Path):
        # Update reference trajectory (simplified mapping)
        N = self.mpc.N
        # We assume the path contains N+1 points at dt intervals
        poses = msg.poses
        num_poses = len(poses)
        
        for k in range(N + 1):
            if k < num_poses:
                self.reference_trajectory[0, k] = poses[k].pose.position.x
                self.reference_trajectory[1, k] = poses[k].pose.position.y
                self.reference_trajectory[2, k] = poses[k].pose.position.z
            else:
                # Hold last position
                if num_poses > 0:
                    self.reference_trajectory[0, k] = poses[-1].pose.position.x
                    self.reference_trajectory[1, k] = poses[-1].pose.position.y
                    self.reference_trajectory[2, k] = poses[-1].pose.position.z
                    
    def control_loop(self):
        start_t = time.perf_counter()
        
        try:
            U_opt, X_opt = self.controller.solve(self.current_state, self.reference_trajectory)
            
            # Publish control commands
            cmd = Wrench()
            cmd.force.z = float(U_opt[0, 0])
            cmd.torque.x = float(U_opt[1, 0])
            cmd.torque.y = float(U_opt[2, 0])
            cmd.torque.z = float(U_opt[3, 0])
            
            self.control_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f"MPC Solve failed: {e}")
            
        dt_solve = (time.perf_counter() - start_t) * 1000
        # self.get_logger().debug(f"Solve time: {dt_solve:.2f} ms")

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
