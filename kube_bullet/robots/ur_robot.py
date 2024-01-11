
from collections import namedtuple, deque
from loguru import logger
import numpy as np
from pybullet_utils.bullet_client import BulletClient

from kube_bullet.robots.robot_base import RobotBase
from kube_bullet.utils.pose_marker import create_pose_marker


JointInfo = namedtuple('JointInfo',
                       ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
                        'maxVelocity', 'controllable'])

class URRobot(RobotBase):

    def __init__(self, 
                 bullet_client: BulletClient, 
                 robot_uid: int,
                 robot_config: dict) -> None:
        
        self.arm_eef_link_idx = 34
        
        self.arm_joint_position = []
        self.waypoints = deque()
        
        self.ik_base_marker_uids = [-1] * 4
        self.ik_eef_marker_uids = [-1] * 4
        
        super().__init__(bullet_client, robot_uid, robot_config)
        
    def initialize_robot(self, reset_joint_positions=True):
        super().initialize_robot(reset_joint_positions=reset_joint_positions)
        
        # add markers
        self.ik_base_marker_uids = create_pose_marker(
            position=[0, 0, 0],
            orientation=[0, 0, 0, 1],
            lifeTime=0, 
            text="base_link_inertia",
            parentObjectUniqueId = self.robot_uid,
            parentLinkIndex=3,
            replaceItemUniqueIdList=self.ik_base_marker_uids
        )
    
    def set_arm_position_controller(self, target_position):
        self.set_joint_position_controller(
            joint_idx=self.motor_joint_indexes,
            position=target_position
        )
    
    def get_joint_states(self):
        # arm: 
        arm_state = self._bc.getJointStates(self.robot_uid,
                                            self.motor_joint_indexes)
                
        self.arm_joint_position = [state[0] for state in arm_state]
       
        return self.arm_joint_position

    def get_eef_pose(self):
        """
        Get eef pose in world coordinate system
        """
        eef_link_state = self._bc.getLinkState(self.robot_uid, self.arm_eef_link_idx)
        eef_pos, eef_qua = eef_link_state[4:6]
        return eef_pos + eef_qua
    
    def get_robot_state(self):
        """
        Overwrite the get_robot_state method in RobotBase
        Add eef pose in world coordinate system
        """
        return {
            'status': self._status,
            'joint_position': self.get_joint_states(),
            'eef_pose': self.get_eef_pose()
        }

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
                    self._status = 'standby'

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
        
        self._status = 'primitive_executing'
        
        return {
            'status': 'primitive_executing',
            'primitive_type': 'primitive_executing',
            'data': {
                'current_joint_goal_position': self.waypoints[-1]
            }
        }
        
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
