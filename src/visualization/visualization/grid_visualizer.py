import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GridMapVisualizer(Node):
    def __init__(self):
        super().__init__('opencv_grid_visualizer')
        self.subscriber_grid_map = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.occupancy_callback,
            10)
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.robot_pose = Pose2D()
        self.publisher_visualized_map = self.create_publisher(
            Image,
            '/visualized_occupancy_map',
            10)
        self.bridge = CvBridge()

    def occupancy_callback(self, msg):
        # Convert OccupancyGrid data to a NumPy array
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        # Print the size of the occupancy grid map
        print(f"Occupancy grid map size: {msg.info.width}x{msg.info.height}")

        # Scale the values for visualization
        image = np.zeros_like(grid, dtype=np.uint8)
        image[grid == -1] = 255  # Unknown spaces to white
        image[grid == 100] = 0   # Occupied spaces to black
        image[grid == 0] = 127   # Free spaces to gray

        # Draw robot pose
        cv2.circle(image, (int(self.robot_pose.x), int(self.robot_pose.y)), 3, (0, 255, 0), -1)
        direction = np.array([np.cos(self.robot_pose.theta), np.sin(self.robot_pose.theta)])
        cv2.arrowedLine(image, (int(self.robot_pose.x), int(self.robot_pose.y)), (int(self.robot_pose.x + direction[0] * 10), int(self.robot_pose.y + direction[1] * 10)), (0, 0, 255), 2)

        # Flip y axis for correct orientation
        flipped_image = np.flipud(image)

        # Publish the image
        ros_image = self.bridge.cv2_to_imgmsg(flipped_image, "mono8")
        self.publisher_visualized_map.publish(ros_image)

        # Display the image using OpenCV
        cv2.imshow("Occupancy Grid", flipped_image)
        cv2.waitKey(1)

        # Print the robot's position on the occupancy grid
        print(f"Robot's position in grid: X={int(self.robot_pose.x)}, Y={int(self.robot_pose.y)}")

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

def main(args=None):
    rclpy.init(args=args)
    visualizer = GridMapVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed
    rclpy.shutdown()

if __name__ == '__main__':
    main()
