
from rclpy.node import Node
import rclpy
import math

# Message imports
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

from tf_transformations import euler_from_quaternion
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class DWACustomPlanner(Node):
    def __init__(self):
        super().__init__("dwa_custom_planner")
        self.get_logger().info("Initializing DWA custom planner")

        #Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        #Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_publisher = self.create_publisher(Marker, "/dwa_trajectories", 10)
        self.marker_id = 0
        self.current_state = {'x': 0.0,
                              'y': 0.0,
                              'yaw': 0.0,
                              'v': 0.0,
                              'w': 0.0}

        self._goal_reached = 1
        self.goal_pose = dict()
        self.lidar_data = None

        #Parameters define as ros parameters later
        self.robot_radius = 0.113
        self.max_forward_speed = 0.1 # m/s
        self.min_forward_speed = -0.1  # m/s
        self.max_yaw_rate = 2.5  # rad/s
        self.max_accel = 0.2  # m/ss
        self.max_delta_yaw_rate = 1.0 # rad/ss
        self.max_decel = 0.2 # m/ss

        self.dt = 0.1
        self.predict_time = 2.0 #second
        self.goal_tolerance = 0.05 #meters
        self.inflation_radius = 0.15#meters
        self.v_samples = 50
        self.w_samples = 50
        
        self.weight_goal = 3.0
        self.weight_heading = 2.0
        self.weight_obstacle = 2.0
        self.weight_turn = 0.2
        self.obstacle_points = []

        self.create_timer(self.dt, self.dwa_control_loop)

    def odom_callback(self, msg: Odometry):
        
        #get and update current robot state
        self.current_state['x'] = msg.pose.pose.position.x
        self.current_state['y'] = msg.pose.pose.position.y
        self.current_state['yaw'] = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        self.current_state['v'] = msg.twist.twist.linear.x
        self.current_state['w'] = msg.twist.twist.angular.z

    def normalize_angle(self, angle):
        """Normalize an angle to be within [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_scan_index(self, angle):
        """Get the index in the LaserScan ranges array corresponding to a given angle."""
        if self.lidar_data is None:
            return -1
        if angle < self.lidar_data.angle_min or angle > self.lidar_data.angle_max:
            return -1
        index = int((angle - self.lidar_data.angle_min) / self.lidar_data.angle_increment)
        if 0 <= index < len(self.lidar_data.ranges):
            return index
        else:
            return -1 # Should not happen
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan data."""
        self.lidar_data = msg
        self.obstacle_points = []
        for i, r in enumerate(msg.ranges):
            if not np.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            self.obstacle_points.append((x, y))
    def calculate_dynamic_window(self):
        """Calculate the dynamic window based on current state and robot constraints."""
        v_possible_min = self.current_state['v'] - self.max_decel * self.dt
        v_possible_max = self.current_state['v'] + self.max_accel * self.dt

        w_possible_min = self.current_state['w'] - self.max_delta_yaw_rate * self.dt
        w_possible_max = self.current_state['w'] + self.max_delta_yaw_rate * self.dt

        v_min = max(self.min_forward_speed, v_possible_min)
        v_max = min(self.max_forward_speed, v_possible_max)

        w_min = max(-self.max_yaw_rate, w_possible_min)
        w_max = min(self.max_yaw_rate, w_possible_max)

        return (v_min, v_max, w_min, w_max)
    
    def _predict_trajectory(self, v, w):
        """Predict the trajectory for given linear and angular velocities."""
        trajectory = []
        x = self.current_state['x']
        y = self.current_state['y']
        theta = self.current_state['yaw']
        n_steps = int(self.predict_time/self.dt)
        for _ in range(n_steps):
            theta = self.normalize_angle(theta + w * self.dt)
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            
            trajectory.append((x, y, theta))
        return trajectory
    def _calculate_goal_cost(self, endpoint):
        """Calculate the cost based on distance to the goal from the trajectory endpoint."""
        goal_x = self.goal_pose['x'] - endpoint[0]
        goal_y = self.goal_pose['y'] - endpoint[1]

        return math.sqrt(goal_x**2 + goal_y**2)
    
    def _calculate_heading_cost(self):
        """Calculate the cost based on the heading difference to the goal."""
        return abs(math.atan2(self.goal_pose['y'] - self.current_state['y'],
                              self.goal_pose['x'] - self.current_state['x']) - self.current_state['yaw'])
    def _turn_cost(self, w):
        """Calculate the cost based on the angular velocity."""
        return abs(w)

    def _calculate_obstacle_cost(self, trajectory):
        """Calculate the cost based on the minimum distance to obstacles along the trajectory."""
        min_dist = float('inf')

        for x, y, _ in trajectory:
            # distance from robot
            d = math.sqrt(x*x + y*y)
            if d < min_dist:
                min_dist = d

        if min_dist < 0.30:  
            return float('inf')

        if min_dist > 1.0:
            return 0.0

        return (1.0 - min_dist)  


    
    def goal_callback(self, msg: PoseStamped):

        """Process incoming goal pose."""
        self.goal_pose['x'] = msg.pose.position.x
        self.goal_pose['y'] = msg.pose.position.y
        self.goal_pose['yaw'] = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])[2]
        self._goal_reached = 0  # Reset goal reached flag
        self.get_logger().info(f"New goal received: x={self.goal_pose['x']}, y={self.goal_pose['y']}, yaw={self.goal_pose['yaw']}")

    def dwa_control_loop(self):
        """Main DWA control loop."""
        best_trajectory = []
        if self.lidar_data is None or self.current_state is None:
            self.get_logger().info("Waiting for data...")
            return
        if self._goal_reached > 0:
            self.get_logger().debug("waiting for goal...")
            return
        #Check if goal is reached
        distance_to_goal = math.sqrt((self.goal_pose['x'] - self.current_state['x'])**2 + (self.goal_pose['y'] - self.current_state['y'])**2)

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info("Goal Reached!")
            self._goal_reached += 1
            stop_msg =Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            return
        #Calculate dynamic Window
        v_min, v_max, w_min, w_max = self.calculate_dynamic_window()

        v_range = np.linspace(v_min, v_max, self.v_samples)
        w_range = np.linspace(w_min, w_max, self.w_samples)

        best_cost = float('inf')
        best_v = 0.0
        best_w = 0.0

        for v in v_range:
            for w in w_range:
                trajectory = self._predict_trajectory(v, w)
                endpoint = trajectory[-1]

                obstacle_cost = self._calculate_obstacle_cost(trajectory)
                if obstacle_cost == float('inf'):
                    continue
                
                goal_cost = self._calculate_goal_cost(endpoint)
                heading_cost = self._calculate_heading_cost()
                turn_cost = self._turn_cost(w)

                total_cost = (self.weight_goal * goal_cost) + (self.weight_heading * heading_cost) + (self.weight_obstacle * obstacle_cost) + (self.weight_turn * turn_cost)

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_w = w
                    best_trajectory = trajectory
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_trajectories"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = []
        for (x, y, theta) in best_trajectory:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        self.marker_publisher.publish(marker)
        if best_cost == float('inf'):
            self.get_logger().info("No valid trajectory found, stopping the robot.")
            stop_msg =Twist()
            self.cmd_vel_publisher.publish(stop_msg)
        else:
            self.get_logger().info("distance to goal: {:.2f}, cmd_vel: v={:.2f}, w={:.2f}".format(distance_to_goal, best_v, best_w))
            cmd_msg = Twist()
            cmd_msg.linear.x = best_v
            cmd_msg.angular.z = best_w
            self.cmd_vel_publisher.publish(cmd_msg)
   
def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWACustomPlanner()
    try:
        rclpy.spin(dwa_node)
    except KeyboardInterrupt:
        dwa_node.get_logger().info("Shutting down planner")
    finally:
        dwa_node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
        