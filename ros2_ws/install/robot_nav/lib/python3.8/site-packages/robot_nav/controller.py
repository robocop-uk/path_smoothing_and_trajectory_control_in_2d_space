import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from robot_nav.trajectory_gen import generate_trajectory
from robot_nav.smoother import smooth_path

LOOKAHEAD_DISTANCE = 0.5
LINEAR_SPEED = 0.2
ANGULAR_GAIN = 2.0
OBSTACLE_THRESHOLD = 0.3  # meters

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.current_pose = None
        self.target_index = 0
        self.obstacle_detected = False

        waypoints = [(0, 0), (1, 2), (3, 3), (5, 2), (6, 0)]
        smooth = smooth_path(waypoints)
        self.trajectory = generate_trajectory(smooth)

        self.timer = self.create_timer(2.0, self.control_loop)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation_q)
        self.current_pose = (position.x, position.y, yaw)

    def scan_callback(self, msg):
        center_ranges = msg.ranges[len(msg.ranges)//3 : 2*len(msg.ranges)//3]
        valid_ranges = [
            r for r in center_ranges
            if 0.05 < r < msg.range_max and not math.isinf(r) and not math.isnan(r)
        ]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.obstacle_detected = min_distance < OBSTACLE_THRESHOLD
        else:
            self.obstacle_detected = False

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.current_pose is None:
            self.get_logger().warn("No current pose yet.")
            return

        x, y, yaw = self.current_pose
        self.get_logger().info(f"Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        if self.obstacle_detected:
            self.get_logger().warn("Obstacle detected! Stopping robot.")
            self.cmd_pub.publish(Twist())
            return

        if self.target_index >= len(self.trajectory):
            self.get_logger().info("Reached end of trajectory. Stopping robot.")
            self.cmd_pub.publish(Twist())
            return

        tx, ty, _ = self.trajectory[self.target_index]
        distance = math.hypot(tx - x, ty - y)

        if distance < LOOKAHEAD_DISTANCE:
            if self.target_index < len(self.trajectory) - 1:
                self.target_index += 1
            else:
                self.get_logger().info("Reached final waypoint. Stopping.")
                self.cmd_pub.publish(Twist())
                return

        dx = tx - x
        dy = ty - y
        target_angle = math.atan2(dy, dx)
        error_angle = self.normalize_angle(target_angle - yaw)

        self.get_logger().info(f"Target: ({tx:.2f}, {ty:.2f}), Heading error: {error_angle:.2f}")

        cmd = Twist()
        cmd.linear.x = LINEAR_SPEED
        cmd.angular.z = ANGULAR_GAIN * error_angle

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Publishing: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

