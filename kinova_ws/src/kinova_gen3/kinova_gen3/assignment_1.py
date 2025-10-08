import rclpy
from rclpy.node import Node
from kinova_gen3_interfaces.srv import SetTool, SetGripper, Status, ClearFaults
import time

class BlockStacker(Node):
    def __init__(self):
        super().__init__("block_stacker")

        # Create clients for the services your interface node provides
        self.clear_faults_client = self.create_client(ClearFaults, "clear_faults")
        self.home_client = self.create_client(Status, "home")
        self.tool_client = self.create_client(SetTool, "set_tool")
        self.gripper_client = self.create_client(SetGripper, "set_gripper")

        # wait for services to be ready
        for c in [self.clear_faults_client, self.home_client, self.tool_client, self.gripper_client]:
            c.wait_for_service()

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
    node = BlockStacker()

    # Step 1: clear faults and home
    node.clear_faults()
    node.set_gripper(0.0)
    node.home()

    # Step 2: move to pickup position (20 cm ahead)
    node.set_tool(0.23, 0.0, 0.04, 0.0, 180.0, 0.0)  # adjust Z and orientation
    time.sleep(3)

    # Step 3: close gripper (value ~ 1.0 for closed, 0.0 for open)
    node.set_gripper(1.0)
    time.sleep(2)
    
    node.set_tool(0.45, 0.0, 0.04, 0.0, 180.0, 0.0)
    time.sleep(2)

    # Step 4: move to stack position
    node.set_tool(0.45, 0.0, 0.02, 0.0, 180.0, 0.0)
    time.sleep(2)

    # Step 5: open gripper
    node.set_gripper(0.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

