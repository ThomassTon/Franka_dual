from franka_dual_msgs.srv import DualJointAngle

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState

DESIRED_ORDER = [
    'left_fr3_joint1', 
    'left_fr3_joint2',
    'left_fr3_joint3',
    'left_fr3_joint4',
    'left_fr3_joint5',
    'left_fr3_joint6',
    'left_fr3_joint7',

    'right_fr3_joint1',
    'right_fr3_joint2',
    'right_fr3_joint3',
    'right_fr3_joint4',
    'right_fr3_joint5',
    'right_fr3_joint6',
    'right_fr3_joint7',

]

class SineWaveJointPublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_joint_publisher')
        self.cli = self.create_client(DualJointAngle, '/franka_dual_arm_joint_position_controller/set_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_listener_callback,
            10
        )
        self.got_inital_joint_state = False
        self.timer = self.create_timer(0.1, self.timer_callback)  # 20 Hz
        self.t = 0.0
        self.freq = 0.5  
        self.amp = 0.2   
        self.njoints = 14  

    def joint_state_listener_callback(self, msg):
        if not self.got_inital_joint_state:
            self.get_logger().info('--- Initial Joint States ---')
            # for name, pos in zip(msg.name, msg.position):
            #     self.get_logger().info(f'Joint {name}: {pos:.3f}')
            self.current_joints = self.get_ordered_joint_positions(msg)
            # print("self.current_joints:  ",self.current_joints)
            self.got_inital_joint_state = True
            # rclpy.shutdown()


    def timer_callback(self):
        if self.got_inital_joint_state:
            self.t += 0.05
            # joints = [self.amp * np.sin(2 * np.pi * self.freq * self.t + i) for i in range(self.njoints)]
            joints = self.current_joints + self.amp * np.sin(2 * np.pi * self.freq * self.t)
            req = DualJointAngle.Request()
            req.data = joints.tolist()
            # req.data = joints
            future = self.cli.call_async(req)
            future.add_done_callback(self.service_response_callback)
            # self.get_logger().info(f'Sent: {["%.3f"%x for x in joints]}')
    
    def service_response_callback(self, future):
        try:
            response = future.result()
            # self.get_logger().info(f"Service response: success={response.success}, message='{response.message}'")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def get_ordered_joint_positions(self, msg, joint_order=DESIRED_ORDER):
        name2pos = dict(zip(msg.name, msg.position))
        return [name2pos.get(joint, 0.0) for joint in joint_order]  # 没有的关节填 0

def main(args=None):
    rclpy.init(args=args)
    node = SineWaveJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

