
import yaml
from loguru import logger
import pybullet as p
from pybullet_utils.bullet_client import BulletClient

# Available robot models
from robots.ur_robot import URRobot
from robots.robotiq_gripper import RobotiqGripper


AVAILABLE_COMPONENTS = {
    'ur10e': URRobot,
    'robotiq_gripper': RobotiqGripper
}


class RobotSpawner:
    
    def __init__(self,
                 bullet_client: BulletClient
                 ) -> None:

        self._bc = bullet_client
        
    def spawn(self,
            robot_name: str,
            robot_config_path: str,
            position: list=[0.0, 0.0, 0.0],
            quaternion: list=[0.0, 0.0, 0.0, 1.0]
            ):
        """
        Initialize the robots
        
        Returns:
            - list of robot instances
        """
        
        self.robot_name = robot_name
        
        self.initial_position = position
        self.initial_quaternion = quaternion
        
        self.robot_config_path = robot_config_path
        self.config = self.parse_robot_config_yaml(robot_config_path)

        # load URDF        
        self.robot_uid = self._bc.loadURDF(
            self.config.get('urdf_path'), 
            position,
            quaternion,
            useFixedBase=True,
            #flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_SELF_COLLISION | p.URDF_USE_MATERIAL_COLORS_FROM_MTL  
            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_MATERIAL_COLORS_FROM_MTL  
        )
        self.print_robot_joint_info(self.robot_uid)
        
        robots = {}
        
        for comp_name in self.config.get('active_components'):
            
            # check component name
            if comp_name not in AVAILABLE_COMPONENTS:
                logger.error(f"Component {comp_name} is not implemented \
                               \n Avaliable components: {AVAILABLE_COMPONENTS.keys()}")

            robot_instance = AVAILABLE_COMPONENTS[comp_name](
                self._bc,
                robot_uid = self.robot_uid,
                robot_config = self.config.get(comp_name)
            )

            robots.update({
                f'{self.robot_name}__{comp_name}':{
                    'instance': robot_instance,
                    'status': 'spawning'
                }
            })
        return robots
    
    def print_robot_joint_info(self, robot_uid):
    
        n_joints = self._bc.getNumJoints(robot_uid)
        for idx in range(n_joints):
            joint_info = self._bc.getJointInfo(robot_uid, idx)
            joint_name = joint_info[1].decode('utf-8')
            logger.info(f"JOINT_{idx}: {joint_name} \n Child name: {joint_info[12]}, index {joint_info[0]}, Type: {joint_info[2]}")
        
    
    @staticmethod
    def parse_robot_config_yaml(yaml_file_path):
                
        with open(yaml_file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
        
        return config

