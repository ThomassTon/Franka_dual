from franka_dual_msgs.srv import DualTraj,EEPose

import rclpy
from std_msgs.msg import Float64MultiArray

from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import torch

from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
import pytorch_kinematics as pk

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

# from pytorch_kinematics.transforms.math import quaternion_close






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

def vector_norm(v):
    return np.linalg.norm(v)

class DualArmCollisionAvoidance(Node):
    def __init__(self):
        super().__init__('dual_arm_collision_avoidance')
        # TF buffer/listener to get EEF positions
        self.joint_state_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.timer_group_tf = MutuallyExclusiveCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trans_l = None
        self.trans_r = None
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1,
            callback_group=self.joint_state_group
        )

        self.timer = self.create_timer(
            0.05,  
            self.control_loop,
            callback_group=self.timer_group
        )
        self.timer_tf = self.create_timer(
            0.05,  
            self.tf_loop,
            callback_group=self.timer_group_tf
        )

        # Publishers for cartesian velocity commands
        self.vel_pub = self.create_publisher(Float64MultiArray, 'target_positions', 10)
        context = LaunchContext()
        urdf_path = PathJoinSubstitution([
            FindPackageShare('panda_dual_basic'), 'config', 'dual_fr3.urdf'
        ]).perform(context)
        self.pk_chain_l = pk.build_serial_chain_from_urdf(open(urdf_path).read(), 'left_fr3_hand_tcp', 'world').to(device= "cuda")
        self.pk_chain_r = pk.build_serial_chain_from_urdf(open(urdf_path).read(), 'right_fr3_hand_tcp', 'world').to(device= "cuda")

        # Define Cartesian goals
        self.goal_left = np.array([0.3, 0.2, 0.5])   # into right workspace
        self.goal_right = np.array([0.3, 0.0, 0.5]) # symmetrical goal

        self.q = None
        # Collision avoidance parameters
        self.safe_dist = 0.3  # meters
        self.k_repulsive = 2  # gain
        self.k_p = 1.0  # proportional gain
        self.k_d = 0.1  # derivative gain

  


    def joint_state_callback(self, msg):

        name2pos = dict(zip(msg.name, msg.position))
        q0 = [name2pos.get(joint, 0.0) for joint in DESIRED_ORDER]
        self.q = np.array(q0)
        # print("get q")

    def jacobian(self,q):
        j_l = self.pk_chain_l.jacobian(q[:7])
        j_r = self.pk_chain_r.jacobian(q[7:])

        return j_l, j_r
    
    def tf_loop(self):
        try:
            now = rclpy.time.Time()
            # Lookup EEF transforms
            print("get tf")
            self.trans_l = self.tf_buffer.lookup_transform('world', 'left_fr3_hand_tcp', now,timeout=Duration(seconds=0.02))
            self.trans_r = self.tf_buffer.lookup_transform('world', 'right_fr3_hand_tcp', now,timeout=Duration(seconds=0.02))
        except Exception:
            print("TF lookup failed:", Exception)
            return

    def control_loop(self):
        # print("get control_loop")
        # try:
        #     now = rclpy.time.Time()
        #     # Lookup EEF transforms
        #     trans_l = self.tf_buffer.lookup_transform('world', 'left_fr3_hand_tcp', now,timeout=Duration(seconds=0.2))
        #     trans_r = self.tf_buffer.lookup_transform('world', 'right_fr3_hand_tcp', now,timeout=Duration(seconds=0.2))
        # except Exception:
        #     print("TF lookup failed:", Exception)
        #     return
        # print("get control_loop")
        # Current positions
        if self.trans_l is not None:

            p_l = np.array([self.trans_l.transform.translation.x,
                            self.trans_l.transform.translation.y,
                            self.trans_l.transform.translation.z])
            p_r = np.array([self.trans_r.transform.translation.x,
                            self.trans_r.transform.translation.y,
                            self.trans_r.transform.translation.z])

   

            # PD control: 
            error_l = self.goal_left - p_l
            error_r = p_r - p_r
            v_attr_l = self.k_p * error_l #- self.k_d * vel_l
            v_attr_r = self.k_p * error_r #- self.k_d * vel_r

            # Repulsive velocity between arms
            diff = p_l - p_r
            dist = vector_norm(diff)
            v_rep = np.zeros(3)
            if dist < self.safe_dist and dist>1e-3:
                rep_weight = (self.safe_dist - dist) / self.safe_dist  # 0~1
                rep_weight = np.clip(rep_weight, 0, 1)
                v_rep = self.k_repulsive * rep_weight * (diff / (dist+1e-6))
            v_rep_max = 0.1
            v_rep = np.clip(v_rep, -v_rep_max, v_rep_max)
            # Left arm: add repulsive if too close
            v_l = v_attr_l + v_rep
            # Right arm: repulsive in opposite direction
            v_r = v_attr_r - v_rep
            if self.q is not None:
                j_l, j_r = self.jacobian(self.q)
                j_l = j_l[0,:3]
                j_r = j_r[0,:3]
                # print(j_l.shape)
                J_l_pinv = torch.linalg.pinv(j_l).cpu().numpy()
                J_r_pinv = torch.linalg.pinv(j_r).cpu().numpy()
                q_l_dot = J_l_pinv @ v_l
                q_r_dot = J_r_pinv @ v_r

                msg = Float64MultiArray()
                msg.data = np.concatenate([q_l_dot, q_r_dot]).tolist()
                self.vel_pub.publish(msg)
       


def main(args=None):
    rclpy.init(args=args)
    node = DualArmCollisionAvoidance()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
