
from loguru import logger
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient

from utils.pose_marker import create_pose_marker
from utils.tf_utils import T


class BulletRenderer:
    """
    Rendering the scene using the Bullet build-in OpenGL-based renderer
    """
    def __init__(self,
                 bullet_client: BulletClient,
                 body_uid: int,
                 link_uid: int,
                 camera_name: str,
                 position=[0.0, 0.0, 0.0],
                 quaternion=[0.0, 0.0, 0.0, 1.0],
                 intrinsic_param = [450, 0, 320, 0, 450, 240, 0, 0, 1],
                 width=640,
                 height=480,
                 depth_range=[0.1, 10],
                 frequence = 30,
                 auto_rendering = True,
                 **kwargs,
                 ) -> None:
        """
        
        :param intrinsic_param (list): the flatted cameraintrinsic matrix
                [[f_x    0     c_x],
                 [0      f_y   c_y],
                 [0      0       1]]
                 assumption: f_x = f_y
        """
        
        # Set defaults if none are received from gRPC
        if len(depth_range) == 0:
            logger.warning(f"Received wrong or empty depth range: {depth_range}, using defaults")
            depth_range = [0.1, 10]
        if len(position) != 3:
            logger.warning(f"Received wrong or empty position offset: {position}, using defaults")
            position = [0.0, 0.0, 0.0]
        if len(quaternion) != 4:
            logger.warning(f"Received wrong or empty rotation offset (quaternion): {quaternion}, using defaults")
            quaternion = [0.0, 0.0, 0.0, 1.0]
        if len(intrinsic_param) != 9:
            logger.warning(f"Received wrong or empty intrinsic parameter: {intrinsic_param}, using defaults")
            intrinsic_param = [450, 0, 320, 0, 450, 240, 0, 0, 1]

        self._bc = bullet_client
        self.camera_name = camera_name
        self.body_uid = body_uid
        self.link_uid = link_uid

        self.t_camera_offset = T(translation=position,
                                 quaternion=quaternion)
        
        self.t_camera_in_world = T()
        
        self.width = width
        self.height = height
        
        self.view_matrix = None
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov = 2 * np.arctan( (height/2) / intrinsic_param[0]) * 180 / np.pi,
            aspect = width / height,
            nearVal = depth_range[0],
            farVal = depth_range[1]
        )

        self.render_freq = frequence
        self.auto_rendering = auto_rendering
        
        self.last_rendering_time = 0

        self.marker_uid = [-1] * 4
        
        self.t_focus_in_camera = T(translation=(0.2, 0.0, 0.0))
        self.t_up_axis_in_camera = np.array([0, 0, 0.2])
        
        logger.info(f"Created camera {self.camera_name}")
        
    def render(self,
               sim_time: str,
               save_images: bool=False):
        """
        Rendering the scene in a certain period of time
        
        :param sim_time: str, current simulation time
        """        
        
        if not self.auto_rendering:
            return
        
        if (sim_time - self.last_rendering_time) < 1 / self.render_freq:
            return
        
        # get current camera pose in world coordinate
        self.get_camera_pose_in_world()
        
        # camera target position - focus point in world coordinate
        t_focus_in_world = self.t_camera_in_world * self.t_focus_in_camera
        # camera up vector (orgin must be the world coordinate's origin point!)
        t_up_axis_in_world = self.t_camera_in_world.matrix[:3, :3] @ self.t_up_axis_in_camera
        
        # calculate view_matrix
        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.t_camera_in_world.translation,
            cameraTargetPosition=t_focus_in_world.translation,
            cameraUpVector=t_up_axis_in_world)

        # get rendered image
        width, height, rgb_img, depth_img, seg_img = self._bc.getCameraImage(
            width=self.width, 
            height=self.width, 
            viewMatrix=self.view_matrix, 
            projectionMatrix=self.proj_matrix,
            shadow=1,
            flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            renderer=p.ER_BULLET_HARDWARE_OPENGL    
        )
        
        # update rendering time stamp
        self.last_rendering_time = sim_time
        
        return rgb_img, depth_img, seg_img

    def get_camera_pose_in_world(self) -> None:
        """
        Get the camera pose in the world coordinate
        """
        if self.body_uid is not None:
            
            camera_link_state = self._bc.getLinkState(
                self.body_uid,
                self.link_uid
            )
            camera_pos, camera_qua = camera_link_state[4:6]
        else:
            # initial pose
            camera_pos = [0.0, 0.0, 0.0]
            camera_qua = [0.0, 0.0, 0.0, 1.0]
        
        self.t_camera_in_world = T(camera_pos, camera_qua) * self.t_camera_offset
        return
        
    def add_pose_marker(self):
        """
        Add debugging marker
        """
        self.marker_uid = create_pose_marker(
            lifeTime=0, 
            text=self.camera_name,
            parentObjectUniqueId=self.body_uid,
            parentLinkIndex=self.link_uid,
            lineLength=0.1,
            replaceItemUniqueIdList=self.marker_uid
        )
    
    def update_parent_uid(self,
                          body_uid,
                          link_uid) -> None:
        """
        Update the camera parent
        """
        self.body_uid = body_uid
        self.link_uid = link_uid
        
        self.add_pose_marker()
