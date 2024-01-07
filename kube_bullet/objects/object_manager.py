
import yaml
from typing import List, Optional, Tuple
from loguru import logger
import pybullet as p
from pybullet_utils.bullet_client import BulletClient

# Build_in objects
# from objects.cube import Cube
from objects.object_from_urdf import ObjectFromURDF


BUILD_IN_OBJECTS = {
}


class ObjectManager:
    """
    This class is responsible for loading and managing the objects in the simulator.
    """
    def __init__(self,
                 bullet_client: BulletClient
                 ):
        
        self._bc = bullet_client
        self._loaded_objs = {}

    def load(self,
            object_name: str,
            config_path: str,
            build_in_object: Optional[str] = None,
            parent_body: Optional[str] = None,
            position: tuple=(0.0, 0.0, 0.0),
            quaternion: tuple=(0.0, 0.0, 0.0, 1.0),
            flags: List[str]= []
            ) -> None:
        """
        Load a interactive object in the simulation environment
        Returns:
            - Object instance
        """
        # load from urdf
        if not build_in_object:
            self.config = self.parse_config_yaml(config_path)
            obj_instance = ObjectFromURDF(
                bullet_client=self._bc,
                urdf_path=self.config.get('urdf_path'),
                object_name=object_name,
                flags=flags,
                parent_body=parent_body,
                init_position=position,
                init_quaternion=quaternion,
            )
            obj_instance.load()
            
            self._loaded_objs[obj_instance.name] = {
                'instance': obj_instance,
                'status': 'loaded',
                'uid': obj_instance.obj_uid
            }
            return

        # load from object class

    def remove(self,
               obj_name: str) -> None:
        """
        Remove object
        """
        if not self.is_object_existed(obj_name):
            logger.warning(f"Object {obj_name} not found!")
            return
        
        # remove markers
        self._loaded_objs[obj_name]['instance'].remove_markers()
        
        obj_uid = self._loaded_objs[obj_name]['uid']
        self._bc.removeBody(obj_uid)
        self._loaded_objs.pop(obj_name)
        
        logger.info(f"Remove object {obj_name}")
        return

    def get_object_state_sequence(self) -> Tuple[str, tuple, tuple]:
        """
        Iterates over all loaded objects and yields the state
        """
        for name, obj in self._loaded_objs.items():
            yield name, *obj['instance'].get_object_pose()

    @property
    def loaded_objects(self):
        """
        Get loaded objects
        """
        return self._loaded_objs
    
    def is_object_existed(self,
                          obj_name: str) -> bool:
        """
        Check whether object is already loaded
        """
        if obj_name in self._loaded_objs.keys():
            return True
        else:
            return False

    @staticmethod
    def parse_config_yaml(
            yaml_file_path: str) -> dict:
        """
        Parse object config yaml file
        """                
        with open(yaml_file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
        
        return config
