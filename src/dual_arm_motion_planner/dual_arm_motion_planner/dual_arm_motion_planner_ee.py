from franka_dual_msgs.srv import DualTraj,EEPose
from ruckig import InputParameter, OutputParameter, Ruckig, Result
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
import time
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
import pytorch_kinematics as pk
import torch
from theseus import SO3
import threading

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

class JointStateReader(Node):
    def __init__(self):
        super().__init__('joint_state_reader')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.future = rclpy.task.Future()
        self.q0 = None

    def joint_state_callback(self, msg):
        name2pos = dict(zip(msg.name, msg.position))
        q0 = [name2pos.get(joint, 0.0) for joint in DESIRED_ORDER]
        self.q0 = np.array(q0)
        self.get_logger().info("Got initial joint state.")
        self.future.set_result(True)


class Planner:
    def __init__(self):
        context = LaunchContext()
        urdf_path = PathJoinSubstitution([
            FindPackageShare('panda_dual_basic'), 'config', 'dual_fr3.urdf'
        ]).perform(context)
        self.pk_chain_l = pk.build_serial_chain_from_urdf(open(urdf_path).read(), 'left_fr3_hand_tcp', 'world').to(device= "cuda")
        self.pk_chain_r = pk.build_serial_chain_from_urdf(open(urdf_path).read(), 'right_fr3_hand_tcp', 'world').to(device= "cuda")
    
    def quintic_coeff(self,q0, qf, T):
        a0 = q0
        a3 = 10*(qf-q0)/T**3
        a4 = -15*(qf-q0)/T**4
        a5 = 6*(qf-q0)/T**5
        return a0, a3, a4, a5

    def sample_traj(self, q0, qf, T, dt=0.1):
        a0,a3,a4,a5 = self.quintic_coeff(q0,qf,T)
        t = np.arange(0,T+dt,dt)
        q = a0 + a3*t**3 + a4*t**4 + a5*t**5
        return t, q

    def forward_kinematics(self, q):
        res_l = self.pk_chain_l.forward_kinematics(q[:7],end_only=True)
        H_l = res_l.get_matrix()
        res_r = self.pk_chain_r.forward_kinematics(q[7:],end_only=True)
        H_r = res_r.get_matrix()

        return H_l[...,:3,-1] ,H_l[:,:3,:3], H_r[...,:3,-1] , H_r[:,:3,:3]
    
    def jacobian(self,q):
        j_l = self.pk_chain_l.jacobian(q[:7])
        j_r = self.pk_chain_r.jacobian(q[7:])
        # print(j_r.shape)
        zeros1 = torch.zeros(( 6, 7), device='cuda')  
        zeros2 = torch.zeros(( 6, 7), device='cuda') 

        J_top = torch.cat([j_l.squeeze(), zeros1], dim=-1)  
        J_bottom = torch.cat([zeros2, j_r.squeeze()], dim=-1) 

        J_combined = torch.cat([J_top, J_bottom], dim=-2)  
        # print("j_c: ",J_combined)
        return J_combined
    
    def rotation_error(self,R_desired, R_current):
        # print(R_desired.shape)
        # print(R_current.shape)
        assert R_desired.shape == R_current.shape, "shape are diff"
        R_err =  R_current @ R_desired.mT  
        _R = SO3()
        _R.update(R_err)
        R_err = _R
        theta_error = R_err.log_map()

        return theta_error
    def gauss_newton_ik_bi(self,q_init, target_pos_l, target_rot_l,target_pos_r, target_rot_r, max_iter=200, tol=3e-3):
        q = q_init
        q_min = torch.tensor([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, 0.5, -2.8973,  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, 0.5, -2.8973]).to('cuda')+0.2
        q_max = torch.tensor([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3, 2.8973,  2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3, 2.8973]).to('cuda') - 0.2
        for i in range(max_iter):
            pos_l, R_ee_l, pos_r, R_ee_r= self.forward_kinematics(q)

            pos_error_l = pos_l - target_pos_l
            rot_error_l = self.rotation_error(target_rot_l, R_ee_l)
            # rot_error_l = target_rot_l - R_ee_l

            pos_error_r = pos_r - target_pos_r
            rot_error_r = self.rotation_error(target_rot_r, R_ee_r)
            # rot_error_r = target_rot_r - R_ee_r
            # print(rot_error_r)
            error = torch.cat([pos_error_l, rot_error_l, pos_error_r,rot_error_r],dim=-1)
            error_norm = torch.linalg.norm(error, dim=-1)
            # print(error_norm)
            # print("error_norm shape",error_norm.shape)
            if error_norm < tol: 
                return q
            J = self.jacobian(q)
            # print("error shape: ",error.shape)
 

            J_damped = J.mT @ torch.inverse(J @ J.mT)
            lr = (100-i)/100
            lr = np.clip(lr, 0.2,1)
            dq =lr* -J_damped @ error[..., None]
            # dq =1* -J_damped @ error[..., None]

            # alpha = 1
            # dq = -torch.linalg.pinv(J) @ error[...,None]  # Gauss-Newton 
            # print(dq)
            q = q + dq.squeeze() 
            q = torch.clip(q, q_min, q_max)  # joint limit
        return None
    def compute_trajectory(self, q0_np):
        q0_torch = torch.from_numpy(q0_np).to(dtype=torch.float,device="cuda",)
        # print("dd")
        pos_l, R_ee_l, pos_r, R_ee_r = self.forward_kinematics(q0_torch)
        pos_l_tar = pos_l+torch.tensor((0.2,0.2,-0.1),device='cuda',dtype=torch.float)
        pos_r_tar = pos_r+torch.tensor((0.2,-0.2,-0.1),device='cuda',dtype=torch.float)
        
        r_l = SO3()   # rpy
        # r_l.update(R_ee_l)

        r_r = SO3()
        # r_r.update(R_ee_r)
   
        R_ee_l_tar = SO3.exp_map(r_l.log_map()+torch.tensor((-torch.pi/2.0,0,-0.))).to_matrix().to("cuda")
        R_ee_r_tar =  SO3.exp_map(r_r.log_map()+torch.tensor((torch.pi/2.0,0,-0.))).to_matrix().to("cuda")




        # print("ee_1", ee_1)
        q = None
        q = self.gauss_newton_ik_bi(q_init=q0_torch,target_pos_l=pos_l_tar, target_rot_l=R_ee_l_tar,target_pos_r=pos_r_tar, target_rot_r=R_ee_r_tar)
        if q is None:
            self.get_logger().error("IK failed.")
            return
        msg = JointTrajectory()
        msg.joint_names = DESIRED_ORDER
        target = q.cpu().numpy()
        # target =q0 +0.1
        # target = q
        T = 3
        for idx, (t_vec, q_vec) in enumerate(
            [self.sample_traj(q0_np[i], target[i], T) for i in range(14)]):
            if idx == 0:  time_vec = t_vec
            qs = q_vec if idx==0 else np.vstack((qs, q_vec))
        for k, tk in enumerate(time_vec):
            pt = JointTrajectoryPoint()
            pt.time_from_start.sec  = int(tk)
            pt.time_from_start.nanosec = int((tk%1)*1e9)
            pt.positions = qs[:,k].tolist()
            msg.points.append(pt)

        req = DualTraj.Request()
        req.trajectory = msg
        return req


class TrajectorySender(Node):
    def __init__(self, req):
        super().__init__('trajectory_sender')
        self.cli = self.create_client(DualTraj, '/franka_dual_arm_joint_traj_controller/set_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = req
        # self.req.trajectory = trajectory_msg

    def send_trajectory(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Trajectory sent successfully.")
        else:
            self.get_logger().error(f"Failed to send trajectory: {future.result().message}")


def main(args=None):
    rclpy.init(args=args)
    
    joint_state_reader = JointStateReader()
    rclpy.spin_until_future_complete(joint_state_reader, joint_state_reader.future)
    q0 = joint_state_reader.q0
    joint_state_reader.destroy_node()

    planner = Planner()
    trajectory_msg = planner.compute_trajectory(q0)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    trajectory_sender = TrajectorySender(trajectory_msg)
    executor.add_node(trajectory_sender)
    executor.spin_once(timeout_sec=1.0)
    trajectory_sender.send_trajectory()
    trajectory_sender.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()