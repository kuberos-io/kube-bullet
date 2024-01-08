
from typing import Optional, List
from loguru import logger

import pybullet as p
from pybullet_utils.bullet_client import BulletClient

from kube_bullet.objects.object_base import ObjectBase



class ObjectFromURDF(ObjectBase):
    """
    Subclass to load an object from URDF file
    """

    def __init__(self,
                 bullet_client: BulletClient,
                 urdf_path: str,
                 object_name: str = 'Unnamed',
                 flags: List[str] = [],
                 parent_body: Optional[str] = None,
                 init_position: tuple = (0.0, 0.0, 0.0),
                 init_quaternion: tuple = (0.0, 0.0, 0.0, 1.0),
                 ) -> None:
        
        super(ObjectFromURDF, self).__init__(
            bullet_client, object_name,
            flags, parent_body,
            init_position, init_quaternion)

        self.urdf_path = urdf_path

    def load(self) -> int:
        """
        Load object from urdf file
        """
        
        self.obj_uid = self._bc.loadURDF(
            self.urdf_path, 
            self._init_position,
            self._init_quaternion,
            flags=p.URDF_USE_INERTIA_FROM_FILE)

        for flag in self._flags:
            if flag == 'SHOW_BASE_POSE':
                self.add_pose_marker(name_postfix='_base')

        logger.info(f"Loaded object {self._obj_name} from urdf: {self.urdf_path}")
        logger.info(f"Base pose in the world: {self.t_base_in_world}")

        return
