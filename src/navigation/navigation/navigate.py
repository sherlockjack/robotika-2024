import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
import numpy as np

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)
        self.timer = self.create_timer(0.1, self.navigate)

        self.robot_pose = Pose2D()
        self.goal_point = Point()
        self.waypoints = Point()
        self.robot_pose_received = False

        
        # Define waypoints
        self.waypoints = [
            Point(x=0.0, y=3.5, z=0.0),
            Point(x=3.5, y=3.5, z=0.0),
            Point(x=-3.5, y=3.50, z=0.0),
            Point(x=0.0, y=3.5, z=0.0),
            Point(x=0.0, y=-3.5, z=0.0),
            Point(x=-3.5, y=-3.5, z=0.0),
            Point(x=-3.5, y=0.5, z=0.0),
            Point(x=-3.5, y=-3.5, z=0.0),
            Point(x=0.0, y=-3.5, z=0.0),
            Point(x=3.5, y=-3.5, z=0.0),
            Point(x=3.5, y=0.5, z=0.0),
            Point(x=3.5, y=-3.5, z=0.0),
        ]
        self.current_waypoint_index = 0
        self.goal_point = self.waypoints[self.current_waypoint_index]
        self.goal_point_received = True

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True
    
    def navigate(self):
        if not self.robot_pose_received or not self.goal_point_received:
            return

        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        print(self.goal_point)
        distance = np.sqrt(dx**2 + dy**2)
        goal_angle = np.arctan2(dy, dx)
        theta = goal_angle - self.robot_pose.theta

        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi

        cmd_vel = Twist()

        if distance > 0.1:
            cmd_vel.linear.y = np.min([0.2 * distance, 0.2])
            cmd_vel.angular.z = 2.0 * theta
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.goal_point = self.waypoints[self.current_waypoint_index]
            else:
                self.goal_point_received = False  # Stop navigation after last waypoint

        self.publisher_cmd_vel.publish(cmd_vel)
        return True
        
def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
