import numpy as np
import pybullet as p
from utils import tf_transformation


class T:
    """
    Transformation matrix class for coordinate transformation
    """
    def __init__(self,
                 translation=(0.0, 0.0, 0.0),
                 quaternion=(0.0, 0.0, 0.0, 1.0),
                 frame_name=None,
                 parent_frame=None,
                 name=None,
                 ) -> None:
        
        self.frame_name = frame_name
        self.parent_frame = parent_frame
        self.name = name
        
        self._matrix = np.zeros((4, 4))
        self._matrix[3, 3] = 1
        
        self._matrix[:3, 3] = np.array(translation)
        self._matrix[:3, :3] = np.array(
            p.getMatrixFromQuaternion(quaternion)).reshape((3, 3))

    @classmethod
    def from_euler(cls, translation, euler):
        
        quaternion = p.getQuaternionFromEuler(euler)
        
        return cls(translation, quaternion)
    
    @classmethod
    def from_matrix(cls, matrix):
        t = cls()
        t._matrix = matrix
        return t
    
    @property
    def translation(self):
        """
        get the translation
        """
        return self._matrix[:3, 3]

    @property
    def matrix(self):
        """
        get the transformation matrix
        """
        return self._matrix
    
    @matrix.setter
    def matrix(self, matrix) -> None:
        self._matrix = matrix
        return
    
    @property
    def quaternion(self):
        """
        get the quaternion
        """
        qua = tf_transformation.quaternion_from_matrix(self._matrix[:4, :4])
        
        return qua
    
    @property
    def euler_rotation(self):
        """
        get the rotation in euler angles
        """
        qua = tf_transformation.quaternion_from_matrix(T[:4, :4])
        euler = p.getEulerFromQuaternion(qua)

        return euler
    
    def __repr__(self) -> str:
        return f"Translation: {self.translation} || Quaternion: {self.quaternion}"
      
    def __str__(self) -> str:
        return f"Translation: {self.translation} || Quaternion: {self.quaternion}"

    def __mul__(self, second_T):
        return T.from_matrix(self.matrix @ second_T.matrix)
