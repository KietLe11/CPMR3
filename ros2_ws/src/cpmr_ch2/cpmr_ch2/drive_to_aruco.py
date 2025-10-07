import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        # These variables will now store the RELATIVE position of the target
        self._target_dist = 0.0      # Z distance from camera
        self._target_sideways = 0.0  # X distance from camera

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._target_info = self.create_subscription(String, "/aruco_target_info", self._target_info_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        self._target_visible = False

    # The target callback now just saves the relative position
    def _target_info_callback(self, msg):
        try:
            if "No targets found!" in msg.data:
                if self._target_visible:
                    self.get_logger().info("Target lost. Searching...")
                self._target_visible = False
                return

            self._target_visible = True
            
            position_str = msg.data.split(']]')[0].strip('[ ')
            x, y, z = map(float, position_str.split())

            # Store the relative distance and sideways error
            self._target_dist = z
            self._target_sideways = -x # Negative so a positive x (left) causes a positive (left) turn
            
        except Exception as e:
            self.get_logger().warn(f"Error processing /aruco_target_info message: '{msg.data}'. Error: {e}")

    # The listener callback is now much simpler
    def _listener_callback(self, msg, fwd_gain=0.5, turn_gain=1.0, stop_dist=0.3, search_speed=0.3):
        twist = Twist()

        if self._target_visible:
            # --- BEHAVIOR 1: TARGET IS VISIBLE ---
            # Drive towards the target based on its relative position
            
            if self._target_dist > stop_dist:
                # We are too far away, so drive forward and turn
                twist.linear.x = self._target_dist * fwd_gain
                twist.angular.z = self._target_sideways * turn_gain
            else:
                # We are close enough, so stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # --- BEHAVIOR 2: TARGET IS LOST ---
            # Rotate in place to search
            twist.linear.x = 0.0
            twist.angular.z = search_speed

        self._publisher.publish(twist)

    # ... (parameter_callback and main function can remain the same) ...
    def parameter_callback(self, params):
        # This function is now less useful as we don't use absolute goals,
        # but we can leave it for now.
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()