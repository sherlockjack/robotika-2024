import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/robot_cmd_vel', 10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        cmd_vel = Twist()
        linear_speed = 1.0  # Kecepatan linear (m/s)
        angular_speed = 1.0  # Kecepatan angular (rad/s)
        try:
            if key.char == 'w':
                cmd_vel.linear.y = linear_speed
            elif key.char == 's':
                cmd_vel.linear.y = -linear_speed
            elif key.char == 'a':
                cmd_vel.angular.z = angular_speed
            elif key.char == 'd':
                cmd_vel.angular.z = -angular_speed
        except AttributeError:
            if key == keyboard.Key.up:
                cmd_vel.linear.y = linear_speed
            elif key == keyboard.Key.down:
                cmd_vel.linear.y = -linear_speed
            elif key == keyboard.Key.left:
                cmd_vel.angular.z = angular_speed
            elif key == keyboard.Key.right:
                cmd_vel.angular.z = -angular_speed
        self.publisher_cmd_vel.publish(cmd_vel)

    def destroy_node(self):
        super().destroy_node()
        self.listener.stop()

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.listener.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
