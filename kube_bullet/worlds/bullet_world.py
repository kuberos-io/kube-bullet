
import os
import pybullet_data
from pybullet_utils.bullet_client import BulletClient



class BulletWorld:
    
    # ASSET_PATH = os.path.join(os.getcwd(), 'assets')
    ASSET_PATH = "/workspace/kube_bullet/assets"

    def __init__(self,
                 bullet_client: BulletClient
                 ) -> None:
        self._bc = bullet_client
        
    def load_assets(self):
        # plane
        self.plane_uid = self._bc.loadURDF(
            os.path.join(pybullet_data.getDataPath(), "plane.urdf"),
            [0, 0, 0]
        )
        
        # table
        # table_pos = [0.0, 0, 0.0]
        # self.table_uid = self.bullet_env.loadURDF(
        #     os.path.join(self.ASSET_PATH, 'bullet', 'table', 'table.urdf'), 
        #     table_pos, [0.0, 0.0, 0.0, 1.0]
        # )
    