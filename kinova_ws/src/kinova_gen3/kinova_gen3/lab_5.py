import rclpy
from rclpy.node import Node
from kinova_gen3_interfaces.srv import SetTool, SetGripper, Status, ClearFaults
import time
from std_msgs.msg import String
import math

class ArmMover(Node):
    def __init__(self):
        super().__init__("arm_mover")

        # Create clients for the services your interface node provides
        self.clear_faults_client = self.create_client(ClearFaults, "clear_faults")
        self.home_client = self.create_client(Status, "home")
        self.tool_client = self.create_client(SetTool, "set_tool")
        self.gripper_client = self.create_client(SetGripper, "set_gripper")
        
        # body parts 
        self.body_parts_dic = {}
        
        # subscription to opencv output 
        self._sub = self.create_subscription(String, 'camera_info', self._listener_callback, 1)

        # wait for services to be ready
        for c in [self.clear_faults_client, self.home_client, self.tool_client, self.gripper_client]:
            c.wait_for_service()
        
        self.clear_faults()
        self.home()
            
    def _listener_callback(self, msg): 
    	data = msg.data  # e.g. "pose_node  LEFT_SHOULDER [5, 329.8869, 381.5413, 0.9301]"
    
        # Split into three parts: node name, body part, and list portion
        parts = data.split(" ", 2)  # ['pose_node', '', 'LEFT_SHOULDER [5, 329.8869, 381.5413, 0.9301]']
    
        # Handle possible double space
        parts = [p for p in parts if p.strip() != '']
    
        node_name = parts[0]
        body_part, rest = parts[1].split(" ", 1)
    
        # Extract numbers from the brackets
        values_str = rest.strip("[]")  # "5, 329.8869, 381.5413, 0.9301"
        values = [float(x.strip()) for x in values_str.split(",")]
    
        kp_id = int(values[0])
        x = values[1]
        y = values[2]
        conf = values[3]
        
        self.body_parts_dic[body_part] = [x, y] 
        
        left_eye = self.body_parts_dic["LEFT_EYE"] 
        left_shoulder = self.body_parts_dic["LEFT_SHOULDER"] 
        right_eye = self.body_parts_dic["RIGHT_EYE"] 
        right_shoulder = self.body_parts_dic["RIGHT_SHOULDER"] 
        left_hand = self.body_parts_dic["LEFT_WRIST"] 
        right_hand = self.body_parts_dic["RIGHT_WRIST"] 
        
        # get distance between left eye and left shoulder 
        left_eye_distance = abs(left_eye[0] - left_shoulder[0])
        
        # get distance between right eye and right shoulder 
        right_eye_distance = abs(right_eye[0] - right_shoulder[0])
        
        # get distance between left hand and left shoulder 
        left_hand_distance = left_hand[1] - left_shoulder[1]
        
        # get distance between right hand and right shoulder 
        right_hand_distance = right_hand[1] - right_shoulder[1])
        
        # both above right hand + left hand + right shoulder + left shoulder > 
        # avg(right eye + right shoulder, left eye + left shoulder) 
        # raise arm to (0, 0.20. 0.10)
        if right_hand_distance > right_eye_distance and left_hand_distance > left_eye_distance: 
        	self.do_set_tool(0, 0.20, 0.10, 0, 180, 0)
        
        # above left hand + left shoulder > left eye + left shoulder
        # raise arm to (0.10, 0.20, 0.10) 
        
        # below left hand + left shoulder > left eye + left shoulder 
        # lower arm to (-0.10, 0.20, 0.10) 
        
        # above right hand + right shoulder > right eye + right shoulder 
        # raise arm to (0, 0.10, 0.10) 
        
        # below right hand + right shoulder > right eye + right shoulder 
        # lower arm to (0, 0.30, 0.10) 
        
        # else don't move 
        

    def clear_faults(self):
        req = ClearFaults.Request()
        future = self.clear_faults_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def home(self):
        req = Status.Request()
        future = self.home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def set_tool(self, x, y, z, theta_x, theta_y, theta_z):
        req = SetTool.Request()
        req.x, req.y, req.z = x, y, z
        req.theta_x, req.theta_y, req.theta_z = theta_x, theta_y, theta_z
        future = self.tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def set_gripper(self, value):
        req = SetGripper.Request()
        req.value = value
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status


def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

