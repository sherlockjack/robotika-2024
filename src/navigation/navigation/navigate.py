import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
from nav_msgs.msg import Path
import numpy as np

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscriber_path = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)
        self.timer = self.create_timer(0.1, self.navigate)

        self.robot_pose = Pose2D()
        self.goal_point = Point()
        self.waypoints = []
        self.current_waypoint_index = 0
        self.robot_pose_received = False
        self.goal_point_received = False

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

    def path_callback(self, msg):
        self.waypoints = [Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.0) for pose in msg.poses]
        print("dasvad")
        print(self.waypoints)
        if self.waypoints:
            self.goal_point = self.waypoints[0]
            self.current_waypoint_index = 0
            self.goal_point_received = True

    def navigate(self):
        if not self.robot_pose_received or not self.goal_point_received:
            return

        dx = self.goal_point.x - self.robot_pose.x -7
        dy = self.goal_point.y - self.robot_pose.y -3
        print("x---")
        print(dx)
        print("y---")
        print(dy)

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
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.goal_point = self.waypoints[self.current_waypoint_index]
            else:
                self.goal_point_received = False  # Stop navigation after last waypoint

        self.publisher_cmd_vel.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
