import rclpy
from rclpy.node import Node
from kinova_gen3_interfaces.srv import SetTool, SetGripper, Status, ClearFaults
import time
from std_msgs.msg import String
import math
import json

class ArmMover(Node):
    def __init__(self):
        super().__init__("arm_mover")

        # Create clients for the services your interface node provides
        self.clear_faults_client = self.create_client(ClearFaults, "clear_faults")
        self.home_client = self.create_client(Status, "home")
        self.tool_client = self.create_client(SetTool, "set_tool")
        self.gripper_client = self.create_client(SetGripper, "set_gripper")
        
        self.busy = False           # prevents command spam
        self.last_move_time = 0.0   # for cooldown timing
        self.cooldown = 1.0    
        
        # body parts 
        self.body_parts_dic = {}
        _BODY_PARTS = ["NOSE", "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR", "LEFT_SHOULDER", "RIGHT_SHOULDER",
                   "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE",
                   "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE"]
        for part in _BODY_PARTS: 
            self.body_parts_dic[part] = []
        
        # subscription to opencv output 
        self._sub = self.create_subscription(String, 'camera_info', self._listener_callback, 1)

        # wait for services to be ready
        for c in [self.clear_faults_client, self.home_client, self.tool_client, self.gripper_client]:
            c.wait_for_service()
        
        self.clear_faults()
        self.home()
            
    def _listener_callback(self, msg): 
        data = json.loads(msg.data)
        for part, (x, y, conf) in data.items():
            self.body_parts_dic[part] = [x, y]
        
        left_eye = self.body_parts_dic["LEFT_EYE"] 
        left_shoulder = self.body_parts_dic["LEFT_SHOULDER"] 
        right_eye = self.body_parts_dic["RIGHT_EYE"] 
        right_shoulder = self.body_parts_dic["RIGHT_SHOULDER"] 
        left_hand = self.body_parts_dic["LEFT_WRIST"] 
        right_hand = self.body_parts_dic["RIGHT_WRIST"] 
        
        def safe_distance(part1, part2, axis=0, absolute=False):
            if part1 != [] and part2 != []:
                d = part1[axis] - part2[axis]
                return abs(d) if absolute else d
            return None

        # vertical distances (y-axis). y increases downward in image coordinates.
        left_eye_distance  = safe_distance(left_eye,  left_shoulder,  axis=1)
        right_eye_distance = safe_distance(right_eye, right_shoulder, axis=1)
        left_hand_distance = safe_distance(left_hand, left_shoulder, axis=1)
        right_hand_distance = safe_distance(right_hand, right_shoulder, axis=1)
        
        # DEBUG: show numeric values for tuning (use debug level or info for short tests)
        self.get_logger().debug(
            f"LEye={left_eye_distance} LHand={left_hand_distance} REye={right_eye_distance} RHand={right_hand_distance}"
        )

        # if all values exist (explicit None check so 0.0 is allowed)
        if None not in [right_hand_distance, right_eye_distance, left_hand_distance, left_eye_distance]:
            now = time.time()
            
            if self.busy or (now - self.last_move_time < self.cooldown):
                return  # skip while moving or during cooldown
                
            # both hands above shoulders (y smaller -> higher in image)
            if right_hand_distance < right_eye_distance and left_hand_distance < left_eye_distance: 
                self.get_logger().info("Detected both hands above shoulders")
                self.set_tool(0, 0.20, 0.10, 0, 180, 0)
        
            # left hand above left shoulder (higher than eye)
            elif left_hand_distance < left_eye_distance:
                self.get_logger().info("Detected left hand over left shoulder")
                self.set_tool(0.10, 0.20, 0.10, 0, 180, 0)                
        
            # right hand above right shoulder
            elif right_hand_distance < right_eye_distance:
                self.get_logger().info("Detected right hand over right shoulder")
                self.set_tool(0, 0.10, 0.10, 0, 180, 0) 
                
            # left hand significantly below left shoulder
            elif left_hand_distance > 0 and abs(left_hand_distance) > abs(left_eye_distance):
                self.get_logger().info("Detected left hand below left shoulder")
                self.set_tool(-0.10, 0.20, 0.10, 0, 180, 0) 
        
            # right hand significantly below right shoulder
            elif right_hand_distance > 0 and abs(right_hand_distance) > abs(right_eye_distance):
                self.get_logger().info("Detected right hand below right shoulder")
                self.set_tool(0, 0.30, 0.10, 0, 180, 0) 
        
            else:
                self.get_logger().info('No motion needed')
        else:
            # When the gate fails, print which values are missing so you know why nothing happens
            self.get_logger().debug(
                f"Missing distance(s). LEye={left_eye_distance} LHand={left_hand_distance} "
                f"REye={right_eye_distance} RHand={right_hand_distance}"
            )

    def clear_faults(self):
        req = ClearFaults.Request()
        future = self.clear_faults_client.call_async(req)
        # keep blocking here at startup is ok, or make async if you want
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def home(self):
        req = Status.Request()
        future = self.home_client.call_async(req)
        # blocking at startup is ok
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    # --- set_tool is now non-blocking (async) ---
    def set_tool(self, x, y, z, theta_x, theta_y, theta_z):
        if self.busy:
            self.get_logger().debug("Ignoring motion command (arm busy)")
            return
        self.busy = True
        req = SetTool.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.theta_x = float(theta_x)
        req.theta_y = float(theta_y)
        req.theta_z = float(theta_z)
        future = self.tool_client.call_async(req)
        # attach a done-callback so we don't block the subscriber thread
        future.add_done_callback(self._on_set_tool_done)
        # return the future if caller wants it
        return future

    def _on_set_tool_done(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"set_tool completed: {res.status}")
        except Exception as e:
            self.get_logger().error(f"set_tool failed: {e}")
        finally:
            self.busy = False
            self.last_move_time = time.time()


    def set_gripper(self, value):
        req = SetGripper.Request()
        req.value = value
        future = self.gripper_client.call_async(req)
        # don't block — attach callback if you care about result
        future.add_done_callback(lambda f: self.get_logger().info("gripper set"))
        return future


def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()

    try:
        rclpy.spin(node)  # <-- keeps the node running and processing subscriber messages
    except KeyboardInterrupt:
        pass  # allow clean Ctrl+C exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

