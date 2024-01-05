
import time
import numpy as np
from loguru import logger

import pybullet as p
from pybullet_utils.bullet_client import BulletClient

from worlds.bullet_world import BulletWorld
from robots.robot_spawner import RobotSpawner
from objects.object_base import ObjectBase
from rendering.renderer_base import BulletRenderer
from utils.pose_marker import create_pose_marker
from utils.tf_utils import T

# grpc
from grpc_kube_bullet.kube_bullet_grpc_server import KubeBulletGrpcServer



class KubeBulletSimulator:
    """
    Kube-Bullet Simulator class based on Pybullet
    """
    
    def __init__(self,
                 use_bullet_gui: bool=True,
                 bullet_time_step: float=1.0/240,
                 rendering_time_step: float=1.0/30,
                 desired_realtime_factor: float=1.0,
                 ) -> None:
        
        self._bullet_time_step = bullet_time_step
        self._rendering_time_step = rendering_time_step
        self._rt_factor = desired_realtime_factor
        
        # start bullet instance
        if use_bullet_gui:
            self._bc = BulletClient(connection_mode=p.GUI)
            self._bc.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
        else:
            self._bc = BulletClient(connection_mode=p.DIRECT)
            
        self._bc.setTimeStep(self._bullet_time_step)
        self._bc.setGravity(0, 0, -9.8)
        
        logger.info("Intialized Bullet instance")
        logger.info(f"Timestep for physic engine: {self._bullet_time_step * 1000} ms")
        
        self.sim_time = 0.0
        
        # robot spawner and grpc server
        self.robot_spawner = RobotSpawner(self._bc)
        self.grpc_server = KubeBulletGrpcServer()
        self.grpc_server.start()
        
        # registered robots, objects, cameras, and debug markers
        self.robot_modules = {}
        self.registered_robot_list = []
        self.interactive_objects = []
        self.cameras = {}
        self.markers = {}
        
        self.initialize_world()

    def setup_update(self) -> None:
        """
        Check the grpc buffer and update simulation scene
        """
        
        cmds = self.grpc_server.get_setup_command()
        while len(cmds) > 0:
            cmd = cmds.popleft()
            logger.info(f"Receive setup command: {cmd}")
            # spawn robot
            if cmd['resource'] == 'robot':
                self.setup_robot(
                    command=cmd['command'],
                    metadata=cmd['metadata'])

            # spawn camera
            if cmd['resource'] == 'camera':
                success, retry, info = self.setup_camera(
                    command=cmd['command'],
                    metadata=cmd['metadata'])
                if retry:
                    self.grpc_server.push_back_unssucceed_commands(cmd)

            # spawn object
            if cmd['resource'] == 'object':
                self.spawn_object(metadata=cmd['cmd_metadata'])

            # add markers
            if cmd['resource'] == 'pose_marker':
                self.create_markers(metadata=cmd['cmd_metadata'])
        return

    def initialize_world(self):
        """
        Initializing the simulation world
        Recommendation: 
          - all static objects, such as tables, shelves, should be added to the world urdf file.
        """
        
        # TODO: Implementation
        self.world = BulletWorld(self._bc)
        self.world.load_assets()
        return

    def setup_robot(self,
                    command: str,
                    metadata: dict) -> None:
        """
        Spawn or remove robot in the simulation
        """
        robot_name = metadata['robot_name']
        
        # check whether the requested robot is already spawned        
        # prevent spawning same robot twice
        if robot_name in self.registered_robot_list:
            if command == 'spawn':
                logger.error(f"Robot [{robot_name}] is already existed in the simulation.")
                return
        else:
            if command == 'remove':
                logger.error(f"Robot [{robot_name}] not existed in the simulation.")
                return

        # spawn robot
        if command == 'spawn':
            spawn_result = self.robot_spawner.spawn(
                robot_name=metadata['robot_name'],
                robot_config_path=metadata['robot_config_path']
            )
            self.robot_modules.update(spawn_result)
            self.registered_robot_list.append(robot_name)
            
            # update grpc buffer TODO
            self.grpc_server.update_setup_execution_status(
                resource='robot',
                name = list(spawn_result.keys()),
                status = ['active'] * len(spawn_result.keys())
            )
            return

        # remove robot
        if command == 'remove':
            # Since the robot_uid is stored in each robot module and the robot has multiple modules
            # therefore, find the first one and call Bullet to remove it. 
            for name, module in self.robot_modules.items():
                if name.split('__')[0] == robot_name:
                    self._bc.removeBody(module['robot_uid'])
                    self.robot_modules.pop(name)

                    # update grpc buffer
                    self.grpc_server.update_setup_execution_status(
                        resource='robot',
                        name = [name],
                        status = ['removed']
                    )
            return


    def setup_camera(self, command: str,
                     metadata: dict) -> None:
        """
        Spawn a camera and attach on the robot in the world
        
        By disabling auto-rendering, you can control camera rendering via a grpc call.
        This function is suitable for generating training data.
        
        :param metadata: dict, request from grpc client, contains metadata for spawning
        :return success: bool, whether spawning success
        :return retry: bool, If retry, push the cmd back to the grpc buffer
        :return info: str, spawn status
        """
        success = False
        retry = False
        
        camera_name = metadata['camera_name']
        if camera_name in [camera['name'] for camera in self.cameras.keys()]:
            logger.error(f"Camera with name [{camera_name}] is already attached in the simulation.")
            return
    
        parent_body = metadata['parent_body']
        parent_link = metadata['parent_link']
        
        # check the robot existence
        if not parent_body in self.robot_modules.keys():
            logger.warning(f"Robot {parent_body} is not existed, waiting for robot spawning... \
                             registered robots: {self.robot_modules.keys()}")
            retry = True
            return success, retry, f"Robot {parent_body} is not existed"
        
        # check link_existence
        body_uid, link_uid = self.robot_modules['ur10e_cell__ur10e']['instance'].get_body_link_uid(
            link_name=parent_link
        )
        if link_uid < 0:
            logger.error(f"Robot link {parent_link} not found")
            return success, retry, f"Robot link {parent_link} not found"
        
        # create camera
        self.cameras.update({
            camera_name: {
                'instance': BulletRenderer(
                    self._bc, 
                    body_uid=body_uid, 
                    link_uid=link_uid,
                    **metadata)
            }
        })
    
        logger.info(f"Add new camera {camera_name}")
        logger.debug(f"All attached cameras: {self.cameras}")
        success = True
        
        return success, retry, ''

    def create_markers(self, metadata=None):
        """
        Create pose markers for debugging purposes.
        
        To update the marker, the older marker will be removed. 
        For better performance we can replace the markers.
        The reason is that for visualizing the eef waypoints, the length of the waypoints is not constant
        """
        
        marker_group_name = metadata['name']
        # check marker group name
        if marker_group_name in list(self.markers.keys()):
            logger.warning(f"Marker group {marker_group_name} is already existed. Remove the old markers")
            self.remove_markers(marker_group_name)
        
        # check parent body
        parent_body = metadata['parent_body']
        parent_link = metadata['parent_link']
        if not parent_body in self.robot_modules:
            logger.error(f"Parent body with name: {parent_body} is not registered.")
            return
        
        # check link name
        body_uid, link_uid = self.robot_modules[parent_body]['instance'].get_body_link_uid(
            link_name=parent_link
        )
        if link_uid < 0:
            logger.error(f"Robot {parent_body} doesn't have link {parent_link}")
            return
        
        # create markers
        self.markers[marker_group_name] = []
        
        for marker in metadata['pose_markers']:
            
            marker_uids = create_pose_marker(
            position=marker.position,
            orientation=marker.quaternion,
            lifeTime=0,
            text=marker.name,
            parentObjectUniqueId = body_uid,
            parentLinkIndex=link_uid,
            lineLength=0.07
            )
            self.markers[marker_group_name].append(marker_uids)
            
        logger.info(f"Create marker group {marker_group_name} with following uids {self.markers}")
        return

    def remove_markers(self, marker_group_name):
        """
        Remove all markers in the given group
        """
        pose_markers = self.markers[marker_group_name]
        for marker in pose_markers:
            for uid in marker:
                self._bc.removeUserDebugItem(uid)
        logger.info(f"Remove markes with uids: {pose_markers}")
        self.markers.pop(marker_group_name)
        return
    
    def spawn_object(self, metadata=None):
        """
        Spawn objects
        TODO: Refactoring
        """
        
        if not metadata is None:
            object_name = metadata['object_name']
            
            if object_name in [obj['name'] for obj in self.interactive_objects]:
                logger.error(f"Object with name [{object_name}] is already existed in the simulation.")
            
            else:
                cube = ObjectBase(
                    self._bc,
                    urdf_path="/workspace/kube_bullet/objects/object_assets/cube/object.urdf"
                )
                cube.load(
                    pos=[0.5, -0.2, 1.0]
                )
        return

    def remove_object(self):
        pass
    
    def renderer_spin(self):
        """
        Call the attached renderer (camera) to render the images
        """
        for _, camera in self.cameras.items():
            camera['instance'].render(sim_time=self.sim_time)
        # self.renderer.render(sim_time=self.sim_time)
        

    def robots_spin(self):
        """
        Check and process the received commands
        NOTE: stepSimulation() is NOT performed.
        """
        
        # check the received robot control command in the grpc buffer
        self.rec_new_control_command()
        
        # check and process the primitive requests
        self.execute_primitives()
        
        # call the robot spin method to update the robot controllers
        for _, rob in self.robot_modules.items():
            rob['instance'].spin()


    def rec_new_control_command(self):
        """
        Get the new control commands such as new target position or velocity
        from grpc buffer.
        """
        # TODO rename
        received_cmds = self.grpc_server.get_robot_commands()

        for robot_cmds in received_cmds:
            # logger.info(f"Robot Commands: {robot_cmds}")
            self.robot_modules[robot_cmds['name']]['instance'].controller_update(
                control_cmds=robot_cmds['control_cmds']
            )
        return
    
    def robots_publish_state(self):
        """
        Publish the robot states in the grpc buffer
        TODO: publish states of all robots and interactive objects
        """
        # get ur_arm_state
        for name, rob in self.robot_modules.items():
            joint_position = rob['instance'].get_joint_states()
            # TODO 
            # for rob_name, joint_position in rob_state.items():
            self.grpc_server.set_robot_joint_state(
                robot_name=name,
                position=joint_position
            )
        return

    def execute_primitives(self):
        """
        Check primitive reqests and processing by calling
        the execute_motion_primitive() in each robot instance.
        """
        
        primitives = self.grpc_server.get_primitives()
        
        for prim in primitives:
            
            if not prim['robot_name'] in self.robot_modules.keys():
                logger.error(f"Request robot {prim['robot_name']} is not rigestered")
                return
            logger.info(f"Processing the motion primitive: {prim}")
            self.robot_modules[prim['robot_name']]['instance'].execute_motion_primitive(
                prim
            )
    
    
    
    def spin(self):
        """
        Main loop: 
            Execute the received commands from the GRPC buffer 
            and step forward in the simulation.
        """     
           
        while 1:
            start = time.time()
            
            self.setup_update()
            
            self.robots_spin()
            
            self.renderer_spin()

            p.stepSimulation()

            self.sim_time += self._bullet_time_step
            
            end = time.time()

            # print debugging info
            if self._rt_factor > 0:
                sleep_time = self._bullet_time_step * (1/self._rt_factor) - (end - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)        
                #     logger.info(f"Sleep_time: {sleep_time * 1000} ms  \
                #         -- Max RT-Factor: {(sleep_time/(end-start)) + 1 }")
                else:
                    logger.warning(f"Execution exceeds desired period. Loop time: {(end - start) * 1000 } ms")
            
            # update the robot states
            self.robots_publish_state()

    def reset(self):
        pass
    
    def shutdown(self):
        pass
