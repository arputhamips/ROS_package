import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')
        
        # State
        self.state = "IDLE" # IDLE, FORWARD, TURN, FINISHED
        self.edge_count = 0
        self.start_pose = None
        self.current_pose = None
        self.current_yaw = 0.0
        
        # Config
        self.target_dist = 1.0  # meters
        self.target_angle = 1.57 # 90 degrees in radians (approx)
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        
        # Pubs/Subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Service to Start Mission
        self.srv = self.create_service(Trigger, 'start_square', self.start_callback)
        
        # Print "Standing By" Message
        self.get_logger().info("Square Controller Initialized.")
        self.get_logger().info("WAITING FOR SERVICE REQUEST: /start_square")

        # Control Loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        # Extract Position
        self.current_pose = msg.pose.pose.position
        
        # Extract Yaw
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def start_callback(self, request, response):
        if self.state == "IDLE" or self.state == "FINISHED":
            self.get_logger().info("Mission Started!")
            self.state = "INIT_FORWARD"
            self.edge_count = 0
            response.success = True
            response.message = "Square path started."
        else:
            response.success = False
            response.message = "Already running!"
        return response

    def control_loop(self):
        # Publish Status constantly so Basestation knows what's up
        status_msg = String()
        status_msg.data = "RUNNING" if self.state not in ["IDLE", "FINISHED"] else self.state
        self.status_pub.publish(status_msg)

        if self.state == "IDLE" or self.state == "FINISHED":
            return
            
        twist = Twist()

        # --- LOGIC STATE MACHINE ---
        if self.state == "INIT_FORWARD":
            self.start_pose = self.current_pose # Capture starting point
            self.state = "FORWARD"
            
        elif self.state == "FORWARD":
            # Calc distance traveled from start_pose
            dx = self.current_pose.x - self.start_pose.x
            dy = self.current_pose.y - self.start_pose.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < self.target_dist:
                twist.linear.x = self.linear_speed
            else:
                twist.linear.x = 0.0
                self.state = "INIT_TURN"
                
        elif self.state == "INIT_TURN":
            self.start_yaw = self.current_yaw # Capture starting heading
            self.state = "TURN"
            
        elif self.state == "TURN":
            # Calc angle turned (handling wrap-around -pi to pi)
            diff = self.current_yaw - self.start_yaw
            # Normalize diff
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            
            if abs(diff) < self.target_angle:
                twist.angular.z = -self.angular_speed # Clockwise
            else:
                twist.angular.z = 0.0
                self.edge_count += 1
                if self.edge_count >= 4:
                    self.state = "FINISHED"
                    self.get_logger().info("Square Completed.")
                else:
                    self.state = "INIT_FORWARD"

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()