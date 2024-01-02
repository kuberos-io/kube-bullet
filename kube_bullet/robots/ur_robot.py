
import copy
from collections import namedtuple, deque
from loguru import logger
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient
from robots.robot_base import RobotBase


JointInfo = namedtuple('JointInfo',
                       ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
                        'maxVelocity', 'controllable'])

class URRobot(RobotBase):
    
    
    def __init__(self, 
                 bullet_client: BulletClient, 
                 robot_uid: str,
                 robot_config: dict) -> None:
        super().__init__(bullet_client, robot_uid, robot_config)
    
        
        self.arm_eef_link_idx = 34
        
        self.arm_joint_position = []
        self.waypoints = deque()
        
        
    def set_arm_position_controller(self, target_position):
        self.set_joint_position_controller(
            joint_idx=self.motor_joint_indexes,
            position=target_position
        )
    
    def get_joint_states(self):
        # arm: 
        arm_state = self._bc.getJointStates(self.robot_uid,
                                            self.motor_joint_indexes)
        
        eef_link_state = self._bc.getLinkState(self.robot_uid, self.arm_eef_link_idx)
        eef_pos, eef_qua = eef_link_state[4:6]
        # logger.info("EEF Pos: {}, Qua: {}", eef_pos, eef_qua)
        
        self.arm_joint_position = [state[0] for state in arm_state]
       
        return self.arm_joint_position

        
    def joint_trajectory_controller_spin(self):
        
        if len(self.arm_joint_position) == 0:
            return
        
        if not len(self.waypoints) == 0:

            # check distance to current goal
            current_goal = self.waypoints[0]
            distance = np.sqrt(np.sum((np.array(current_goal) - np.array(self.arm_joint_position)) ** 2))
            if distance > 0.02:
                self.set_arm_position_controller(current_goal)
            else:
                              
                self.waypoints.popleft()
                logger.info(f"Reach the waypoint: {current_goal}, \
                              Number of remained waypoints: {len(self.waypoints)}")
                
                # check whether reaching the final goal
                if len(self.waypoints) == 0:
                    self.set_motion_execution_status('finished')
                    logger.info(f"trajectory execution finished")
                    
                # spin again in one loop to update the postion_controller
                self.joint_trajectory_controller_spin()


    def spin(self):
        """
        Select the controllers:
         - using joint_trajectory_controller to execte all motion_primitives
         - otherwise using joint_position_controller to update motor_joint_positions
        
        """
        if self.motion_execution_status in ['executing']:
            self.joint_trajectory_controller_spin()
            # logger.info(f"Motion Execution Status: {self.motion_execution_status}")
        
    def move_eef_through_poses(self, data) -> None:
        """
        Primitive: move the eef through poses in robot base coordinate system
        
        TODO: 
            - change the world coordinate to the robot cooridnate system
            - remove the fixed orientation
        """
        
        pos = data['eef_pos']
        euler = data['eef_euler']
        
        self.set_motion_execution_status('executing')
        
        self.waypoints = deque()
        
        qua = [-0.7070952653884888, 0.7071177959442139, 0.0008148834458552301, 0.0]
        
        for i in range(len(pos)):
            self.waypoints.append(self.calculate_eef_ik(pos[i], qua))
        
        return 
        
    def calculate_eef_ik(self, position, quaternion):
        """
        Compute the ik by the given end effetor pose
        
        Returns: 
            joint_position (tuple): robot joint positions (Filtered by robot joint index)
        """
        
        joint_positions = self._bc.calculateInverseKinematics(
            self.robot_uid,
            self.arm_eef_link_idx,
            position,
            quaternion,
            lowerLimits=[-2 * np.pi] * 6,
            upperLimits=[2 * np.pi] * 6,
            jointRanges=[4 * np.pi] * 6,
            restPoses=list(np.array([0, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi),
            maxNumIterations=10000,
            residualThreshold=1e-5
        )
        
        robot_joint_positions = joint_positions[0:6]
        
        # logger.info("JP: {}", joint_positions)
        
        return robot_joint_positions
    
