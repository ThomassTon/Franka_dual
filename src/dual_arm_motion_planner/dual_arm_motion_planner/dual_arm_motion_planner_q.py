from franka_dual_msgs.srv import DualTraj
from ruckig import InputParameter, OutputParameter, Ruckig, Result
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
import time
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

class Join_Traj_Plan(Node):
    def __init__(self):
        super().__init__('join_traj_plan_publisher')
        self.cli = self.create_client(DualTraj, '/franka_dual_arm_joint_traj_controller/set_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_listener_callback,
            10
        )
        self.got_inital_joint_state = False

    def joint_state_listener_callback(self, msg):
        if not self.got_inital_joint_state:
            self.get_logger().info('--- Initial Joint States ---')
            self.current_joints = self.get_ordered_joint_positions(msg)
            self.got_inital_joint_state = True
            self.traj_plan()

    
    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: success={response.success}, message='{response.message}'")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def get_ordered_joint_positions(self, msg, joint_order=DESIRED_ORDER):
        name2pos = dict(zip(msg.name, msg.position))
        return [name2pos.get(joint, 0.0) for joint in joint_order]  
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
    
    def traj_plan(self,target= None):
        if self.got_inital_joint_state:
            q0 = np.array(self.current_joints)
            msg = JointTrajectory()
            msg.joint_names = DESIRED_ORDER
            target = q0 + 0.5
            T = 3
            for idx, (t_vec, q_vec) in enumerate(
                [self.sample_traj(q0[i], target[i], T) for i in range(14)]):
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
            future = self.cli.call_async(req)
            future.add_done_callback(self.service_response_callback)
            self.get_logger().info('Sent Trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = Join_Traj_Plan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()