
from typing import List, Optional, Tuple
from pybullet_utils.bullet_client import BulletClient

from utils.pose_marker import create_pose_marker
from utils.tf_utils import T


class ObjectBase:
    """
    Base class for representing an object in the simulator
    """
    def __init__(self,
                 bullet_client: BulletClient,
                 object_name: str = 'Unnamed',
                 flags: List[str] = [],
                 parent_body: Optional[str] = None,
                 init_position: tuple = (0.0, 0.0, 0.0),
                 init_quaternion: tuple = (0.0, 0.0, 0.0, 1.0),
                 ) -> None:
        
        # set default initial pose, if get empty tuple
        if len(init_position) !=3:
            init_position = (0.0, 0.0, 0.0)
        if len(init_quaternion) !=4:
            init_quaternion = (0.0, 0.0, 0.0, 1.0)
        
        self._bc = bullet_client
        self._obj_name = object_name
        self._flags = flags
        
        self._parent_body = parent_body
        self._init_position = init_position
        self._init_quaternion = init_quaternion
        
        self.t_base_in_world = T(
            self._init_position,
            self._init_quaternion,
            frame_name=f"{self._obj_name}_base",
            parent_frame='world'
        )    
        
        self.base_marker_uid = [-1] * 4

    def load(self) -> dict:
        """
        This method should be implemented by Subclass
        """
        return NotImplementedError("Load method should be implemented by the subclass")

    def add_pose_marker(self,
                        name_postfix: str ='') -> None:
        """
        Add 3d coordinate marker for debugging
        """    
        self.base_marker_uid = create_pose_marker(
            lifeTime=0, 
            text=f'{self._obj_name}{name_postfix}',
            parentObjectUniqueId=self.obj_uid,
            lineLength=0.1,
            replaceItemUniqueIdList=self.base_marker_uid
        )
        return

    @property
    def name(self):
        """Get object name"""
        return self._obj_name
    
    def reset(self):
        """
        Reset the object base position and orientation
        """
        self._bc.resetBasePositionAndOrientation(
            self.obj_uid,
            posObj=self._init_position,
            ornObj=self._init_quaternion,
        )
        return

    def remove_markers(self) -> None:
        """Remove pose markers"""
        for uid in self.base_marker_uid:
            self._bc.removeUserDebugItem(uid)
        return

    def get_object_pose(self) -> Tuple[list, list]:
        """
        Get object pose in world coordinate
        """
        pos, qua = self._bc.getBasePositionAndOrientation(
            self.obj_uid
        )
        return pos, qua    

    def set_object_pose(self,
                        pos: list,
                        qua: list) -> None:
        """Set object base pose"""
        self._bc.resetBasePositionAndOrientation(
            self.obj_uid,
            posObj=pos,
            ornObj=qua,
        )
