import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from robot_nav.trajectory_gen import generate_trajectory
from robot_nav.smoother import smooth_path

LOOKAHEAD_DISTANCE = 0.5
LINEAR_SPEED = 0.2
ANGULAR_GAIN = 1.5

class PurePursuitController(Node):
    def __init__(self, trajectory):
        super().__init__('pure_pursuit_controller')
        self.trajectory = trajectory
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_pose = None
        self.target_index = 0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_pose = (x, y, yaw)

    def control_loop(self):
        if self.current_pose is None:
            return
        x, y, yaw = self.current_pose
        while self.target_index < len(self.trajectory):
            tx, ty, _ = self.trajectory[self.target_index]
            if math.hypot(tx - x, ty - y) >= LOOKAHEAD_DISTANCE:
                break
            self.target_index += 1
        if self.target_index >= len(self.trajectory):
            self.cmd_pub.publish(Twist())
            return
        tx, ty, _ = self.trajectory[self.target_index]
        dx = tx - x
        dy = ty - y
        target_angle = math.atan2(dy, dx)
        error_angle = self.normalize_angle(target_angle - yaw)
        cmd = Twist()
        cmd.linear.x = LINEAR_SPEED
        cmd.angular.z = ANGULAR_GAIN * error_angle
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rclpy.init()
    waypoints = [(0, 0), (1, 1), (2, 2), (3, 1), (4, 0)]
    smooth = smooth_path(waypoints)
    trajectory = generate_trajectory(smooth)
    controller = PurePursuitController(trajectory)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
