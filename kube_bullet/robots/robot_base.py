
from collections import namedtuple
from typing import Optional, Tuple
from loguru import logger
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient


BulletJointInfo = namedtuple('BulletJointInfo',
    ['index', 'name', 'type', 'damping', 'friction',
     'lowerLimit', 'upperLimit', 'maxForce',
    'maxVelocity', 'controllable'])


class RobotBase:
    """
    Base robot class contains basic method load and initialize a robot from 
    the configuration yaml file and provides following metheds
    
     - find controllable joint indexes
     - initialize: hard reset joint position and set joint position controller
     - execute_motion_primitive: primitive need to be implemented in child class
     - joint_position_controller
     - joint_velocity_controller
    """
    
    def __init__(self,
                 bullet_client: BulletClient,
                 robot_uid: int,
                 robot_config: dict) -> None:
        """
        Initialize the robot
        
        :param bullet_client:
        :param robot_uid: int, robot uid in bullet returned by loadURDF
        :param robot_config: dict, robot component configuration from yaml file
        """
        
        self._bc = bullet_client
        self.robot_uid = robot_uid
        self.config = robot_config

        # find the indexes of controllable joints
        joint_names = self.config.get('joints')
        
        self.motor_joints = self.find_joint_index(joint_names,
                                                  check_controllable=True)
        logger.info(f"Found active joints: {self.motor_joints}")
        
        self.motor_joint_indexes = list(self.motor_joints.values())
        logger.info(f"Motor joint indexes: {self.motor_joint_indexes}")
        
        # ATTRIBUTES
        self._status = 'loaded'
        self._motion_execution_status = 'none'

        self.initialize_robot()
        
    def initialize_robot(self, reset_joint_positions=True):
        """
        Initializing robot
            - hard reset the joint position
            - reset joint_position_controller
        """
        self.set_status('initializing')
        
        # reset
        if reset_joint_positions:
            for i in range(len(self.motor_joint_indexes)):
                self._bc.resetJointState(
                    self.robot_uid,
                    self.motor_joint_indexes[i],
                    self.config['initial_positions'][i])

        # set joint motor controller
        self.set_joint_position_controller(
            joint_idx=self.motor_joint_indexes,
            position=self.config['initial_positions'],
        )

        logger.info(f"Initialized: {self.config}.")

        self.set_status('standby')


    def find_joint_index(self,
                         joint_names: list,
                         check_controllable=False) -> dict:
        """
        Find the controllable joint indexes based on the configuration
        """
        
        joints = {}
        
        n_joints = self._bc.getNumJoints(self.robot_uid)
        
        for i in range(n_joints):
            # get joint info
            j_info = list(self._bc.getJointInfo(self.robot_uid, i))
            j_info = BulletJointInfo(
                j_info[0], j_info[1].decode("utf-8"), 
                j_info[3], *j_info[6:12], 
                j_info[2] != p.JOINT_FIXED
            )
            # find joint index
            if j_info.name in joint_names:
                if check_controllable:
                    if not j_info.controllable:
                        logger.error(f"Given joint name {j_info['']}")
                joints.update({
                    j_info.name: j_info.index
                })
                
        return joints
    
    def get_body_link_uid(self,
                          link_name: str
                          ) -> Tuple[Optional[int], Optional[int]]:
        """
        Find the link uid by given its name.
        
        :param link_name: str, the name of the link to find.
        :return link_uid: (str, None), link uid or None if not found.
        """
        
        n_joints = self._bc.getNumJoints(self.robot_uid)
        
        for link_uid in range(n_joints):
            # get joint info
            j_info = list(self._bc.getJointInfo(self.robot_uid, link_uid))
            if j_info[12].decode('UTF-8') == link_name:
                return self.robot_uid, link_uid
            
        return None, None
    
    
    def controller_update(self,
                          control_cmds: dict) -> None:
        
        for cmd_type, cmd_value in control_cmds.items():
            if cmd_type == 'joint_position_control':
                self.set_joint_position_controller(
                    self.motor_joint_indexes,
                    position=cmd_value
                )
        
    
    def execute_motion_primitive(self, primitive: dict) -> None:
        """
        Execute the customized motion primitives that implemented by robot class,
        such as move eef through poses
         
        This method that called in the simulation loop
        
        """
        getattr(self, primitive['primitive_type'])(primitive['data'])
        self.set_motion_execution_status('executing')
        return
    
    def reset(self):
        pass
    
    
    def set_joint_position_controller(self, joint_idx, position):
        """
        Set the new target position for robot joints
        """
        
        self._bc.setJointMotorControlArray(
            bodyIndex=self.robot_uid,
            jointIndices=self.motor_joint_indexes,
            controlMode=p.POSITION_CONTROL,
            targetPositions=position,
            positionGains=np.ones(len(self.motor_joint_indexes)) * 0.005,
        )
    
    @property
    def status(self):
        """
        Getter for the status
        """
        return self._status
    
    # @status.setter
    def set_status(self, new_status):
        """
        Setter to update the status
        """
        if new_status in ['initializing', 'standby', 'reset']:
            self._status = new_status
        else:
            logger.error(f"Invalid status: {new_status}")
    
    @property
    def motion_execution_status(self):
        """
        Getter for the motion execution status
        """
        return self._motion_execution_status
    
    # @motion_execution_status.setter
    def set_motion_execution_status(self, new_status):
        """
        Setter to update the status
        """
        if new_status in ['none', 'executing', 'aborted', 'finished']:
            self._motion_execution_status = new_status
        else:
            logger.error(f"Invalid motion execution status: {new_status}")

