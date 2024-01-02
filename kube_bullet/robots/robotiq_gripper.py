
from collections import namedtuple
from loguru import logger
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient
from robots.robot_base import RobotBase


JointInfo = namedtuple('JointInfo',
                       ['id', 'name', 'type', 'damping', 'friction', 'lowerLimit', 'upperLimit', 'maxForce',
                        'maxVelocity', 'controllable'])


class RobotiqGripper(RobotBase):
    
    GRASPING_FORCE = 5
    
    def __init__(self, 
                 bullet_client: BulletClient, 
                 robot_uid: str,
                 robot_config: dict) -> None:
        super().__init__(bullet_client, robot_uid, robot_config)
    
        self.joint_idx = [26]
        
        self.setup_robotiq_gripper()

    
    def get_joint_states(self):
        # gripper
        gripper_state = self._bc.getJointStates(self.robot_uid,
                                                self.joint_idx)
        
        return [state[0] for state in gripper_state]
        

    def spin(self):
        return
    
    def simple_gripper_control(self, data):
        
        if data['gripper_action'] == 'close':
            logger.info('Close gripper')
            self.close()
        elif data['gripper_action'] == 'open':
            logger.info("Open gripper")
            self.open(
                position=data['position']
            )


    def setup_robotiq_gripper(self):
        
        mimic_parent_name = self.config.get('mimic_parent')
        mimic_joints = self.config.get('mimic_joints')
        
        mimic_joint_names = list(mimic_joints.keys())
        mimic_joint_indexes_dict = self.find_joint_index(mimic_joint_names)
        
        mimic_joint_indexes = []
        mimic_joint_gains = []
        for name in mimic_joint_names:
            mimic_joint_indexes.append(mimic_joint_indexes_dict[name])
            mimic_joint_gains.append(mimic_joints[name]['gain'])
        
        # create constraints
        for i in range(len(mimic_joint_names)):
            logger.info(f"Setup gripper mimic \n \
                      Parent joint: {mimic_parent_name} \n \
                      Child joint: {mimic_joint_names[i]} \n \
                      Child joint index {mimic_joint_indexes[i]} \n \
                      Gain: {mimic_joint_gains[i]}")
            
            constraint = self._bc.createConstraint(self.robot_uid, self.motor_joint_indexes[0],
                                                    self.robot_uid, mimic_joint_indexes[i],
                                                    jointType=p.JOINT_GEAR,
                                                    jointAxis=[0, 1, 0],
                                                    parentFramePosition=[0, 0, 0],
                                                    childFramePosition=[0, 0, 0])
            self._bc.changeConstraint(
                constraint,
                gearRatio=mimic_joint_gains[i],
                maxForce=10,
                erp=1)
        
        
        # gripper joints
        self.gripper_joint_indexes = [self.motor_joint_indexes[0], *mimic_joint_indexes]
        # TODO: Check the mimic with constraints
        self.gripper_joint_gains = [1, *[-gain for gain in mimic_joint_gains]]
        
        logger.info(f"All gripper joint indexes: {self.gripper_joint_indexes}")
        logger.info(f"Gains for mimic joints: {self.gripper_joint_gains}")
        
        self.reset_gripper_joints()
        
        logger.info("Gripper loaded")
    
    def reset_gripper_joints(self):
        
        initial_position = self.config['initial_positions'][0]
        # reset joint
        for i in range(len(self.gripper_joint_indexes)):
            self._bc.resetJointState(
                self.robot_uid,
                self.gripper_joint_indexes[i],
                self.gripper_joint_gains[i] * initial_position #  * (-1)
            )
            
        # update motor controller
        self._bc.setJointMotorControlArray(
            bodyIndex=self.robot_uid,
            jointIndices=self.gripper_joint_indexes,
            controlMode=p.POSITION_CONTROL,
            targetPositions=np.array([1.0, 1.0, -1.0, 1.0]) * initial_position, # TODO check the mimic gains
            # force=self.GRASPING_FORCE,
            # positionGain=0.1
        )


    def open(self, position):
        
        logger.info("Opening gripper")

        self._bc.setJointMotorControlArray(
            bodyIndex=self.robot_uid,
            jointIndices=self.gripper_joint_indexes, # [26, 28, 30, 32],
            controlMode=p.POSITION_CONTROL,
            targetPositions=np.array([1.0, 1.0, -1.0, 1.0]) * position,
            # force=self.GRASPING_FORCE,
            # positionGain=0.1
        )

    
    def close(self):
        
        logger.info("Closing gripper")
        
        self._bc.setJointMotorControlArray(
            bodyIndex=self.robot_uid,
            jointIndices=self.gripper_joint_indexes, # [26, 28, 30, 32],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[1.0, 1.0, -1.0, 1.0],
            forces=[1, 1, 1 ,1]
        )
        
    

        

    # def reset_joint_for_testing(self):
        
        # self._bc.resetJointState(
        #             self.robot_uid,
        #             27,
        #             0.1)
        
        # self._bc.resetJointState(
        #             self.robot_uid,
        #             29,
        #             0.1)
        
        # self._bc.resetJointState(
        #             self.robot_uid,
        #             31,
        #             -0.1)
        
        # self._bc.resetJointState(
        #             self.robot_uid,
        #             33,
        #             0.1)
        
        # self._bc.setJointMotorControlArray(
        #     bodyIndex=self.robot_uid,
        #     jointIndices=[27, 29, 31, 33],
        #     controlMode=p.POSITION_CONTROL,
        #     targetPositions=[0.1, 0.1, -0.1, 0.1],
        #     # force=self.GRASPING_FORCE,
        #     # positionGain=0.1
        # )
        
    
        # def setup_joints(self):
        # for joint_id, multiplier in self.mimic_child_multiplier.items():
        #     c = self._bc.createConstraint(self.robot_uid, self.mimic_parent_id,
        #                                             self.robot_uid, joint_id,
        #                                             jointType=p.JOINT_GEAR,
        #                                             jointAxis=[0, 1, 0],
        #                                             parentFramePosition=[0, 0, 0],
        #                                             childFramePosition=[0, 0, 0])
        #     self._bc.changeConstraint(c, gearRatio=multiplier, maxForce=10, erp=1)
