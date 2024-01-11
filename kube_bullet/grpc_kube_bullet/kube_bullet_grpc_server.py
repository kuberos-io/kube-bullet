
import time
import numpy as np
from loguru import logger
from typing import List, Optional
import copy
import grpc
from grpc import server
from concurrent import futures 
import threading
from collections import deque

from . import kube_bullet_grpc_pb2
from . import kube_bullet_grpc_pb2_grpc

from kube_bullet.utils.tf_utils import T


class _KubeBulletServicer(kube_bullet_grpc_pb2_grpc.KubeBulletInterfaceServicer):
    """
    This class serves as a gRPC servicer for the KubeBullet interface, providing 
    functionality for simulation and robot control in a Bullet physics environment.
    
    It implements the gRPC protobuf definitions, to handle incoming gRPC requests.

    """
    def __init__(self, cb=None) -> None:
        """
        :param cb (callable, optional): An optional callback function for extra processing.
        """
        self._cb = cb # CHECK
        
        self.new_sim_control_cmd = False
        self.run_steps = -1 # -1 means run continuously
        self.rt_factor = 1
        self.run_timestep = 1.0/240
        
        # cache all simulation setup commands
        self.sim_setup_cmds = deque()
        
        # cache the state and control commands of all registered robot modules
        self.robot_modules = {}
        
        # cache the state of loaded objects
        self.object_state = {}
        
        # cache all control primitive (robot component level) request
        self.control_primitives = []
        
        # cache task-level skill primitives
        self.skill_primitives = []
        
        super().__init__()
    
    def CheckServerConnection(self, request, context):
        """
        Check connection status (TCP) for client before sending functional calls
        """
        return kube_bullet_grpc_pb2.Pong(pong = True)

    ################## Simulation Setup ##################
    def SetupSimulator(self, request, context):
        """
        Setup simulator
        """
        logger.info(f"Receive simulator control command: {request}")
        
        if request.action == 'RUN_RT':
            self.run_steps = -1
            self.rt_factor = 1.0
        if request.action == 'RUN_FAST':
            self.run_steps = -1
            self.rt_factor = 0.0
        
        if request.action == 'RUN_STEPS' and request.steps:
            self.run_steps = request.steps
            self.rt_factor = 1.0
            
        if request.action == 'STOP':
            self.run_steps = 0
            self.rt_factor = 1.0
        
        if request.timestep > 0:
            self.run_timestep = request.timestep 
        else:
            logger.info(f"Using current timestep: {self.run_timestep * 1000 } ms")
        
        self.new_sim_control_cmd = True
        return kube_bullet_grpc_pb2.SimulatorStatus(
            rt_factor = self.rt_factor
        )

    def SetupRobot(self, request, context):
        """Process robot setup request"""
        cmd = {
            'resource': 'robot',
            'command': 'spawn',
            'metadata': {
                'robot_name': request.robot_name,
                'robot_config_path': request.robot_config_path,
                'active_components': request.active_components,
            }
        }
        self.sim_setup_cmds.append(cmd)

        # register robot controllable modules
        for comp_name in request.active_components:
            rob_module_name = f'{request.robot_name}__{comp_name}'
            if not rob_module_name in self.robot_modules:
                self.robot_modules.update({
                    rob_module_name: {
                        'status': 'spawning',
                        'joint_position': [],
                        'joint_position_cmd': [],
                        'joint_velocity': [],
                        'eef_pose': [],
                        'current_eef_goal_position': [],
                        'current_eef_goal_quaternion': [],
                        'current_joint_goal_position': [],
                        'is_new_command': False,
                    },
                })
        logger.info(f"Received robot setup request: {request}")
        
        # TODO: Use a new proto message? 
        #       Because of the inconsistency between robot_name and robot_module_name
        return kube_bullet_grpc_pb2.RobotStatus(
            robot_module_name = request.robot_name,
            status = 'spawning',
        )
    
    def SetupCamera(self, request, context):
        """Process camera setup request"""
        cmd = {
            'resource': 'camera',
            'command': request.command,
            'metadata': {
                'camera_name': request.camera_name,
                'parent_body': request.parent_body,
                'parent_link': request.parent_link,
                'auto_rendering': request.auto_rendering,
                'position': request.position,
                'quaternion': request.quaternion,
                'intrinsic_param': request.intrinsic_param,
                'depth_range': request.depth_range
            }
        }
        self.sim_setup_cmds.append(cmd)
        
        return kube_bullet_grpc_pb2.CameraStatus(
            camera_name = request.camera_name,
            status = 'spawning')

    def SetupObject(self, request, context):
        
        cmd = {
            'resource': 'object',
            'command': request.command,
            'metadata': {
                'object_name': request.object_name,
                'config_path': request.config_path,
                'build_in_object': request.build_in_object,
                'parent_body': request.parent_body,
                'position': request.position,
                'quaternion': request.quaternion,
                'flags': request.flags,
            }
        }
        self.sim_setup_cmds.append(cmd)

        self.object_state.update({
            request.object_name: {
                'status': 'LOADING',
                'position': None,
                'quaternion': None
            }
        })

        return kube_bullet_grpc_pb2.ObjectState(
            object_name = request.object_name,
        )


    ################## Robot Control Interface ##################
    def SendRobotJointPositionCommand(self, request, context):
        """
        Receive the joint position control command and return curresnt state
        """
        # logger.debug(f'Received joint position command: {request}')
        # check existence of robot module
        try:
            self.robot_modules[request.robot_module_name]['joint_position_cmd'] = request.positions
            self.robot_modules[request.robot_module_name]['is_new_command'] = True

            robot_status = kube_bullet_grpc_pb2.RobotStatus(
                joint_state = kube_bullet_grpc_pb2.RobotJointState(
                    positions = self.robot_modules[request.robot_module_name]['joint_position']
                )
            )
        except KeyError:
            robot_status = kube_bullet_grpc_pb2.RobotStatus()
            robot_status.status = f"[ERROR] robot module {request.robot_module_name} not found"

        return robot_status


    ################## Control primitives ##################
    def MoveArmThroughEefPoses(self, request, context):
        """
        Request to move the end-effector through a list of poses
        with out feedback
        """
        eef_poses, eef_eulers = self.convert_pose_msg_to_list(request.eef_poses)

        self.control_primitives.append({
            'robot_name': request.robot_module_name,
            'primitive_type': 'move_eef_through_poses',
            'data': {
                'eef_pos': eef_poses,
                'eef_euler': eef_eulers
            }
        })
        logger.info(f"Received Primitives {self.control_primitives}")
        
        robot_status = kube_bullet_grpc_pb2.RobotStatus()
        robot_status.status = 'Accepted'
        return robot_status


    def MoveArmThroughEefPosesWithFeedback(self, request, context):
        """
        Request to move the end-effector through a list of poses
        Return the current eef pose and remaining distance to the goal as continuous feedback
        """
        logger.info(f"Received control primitive: Move EEF through poses with feedback: {self.control_primitives}")

        eef_poses, eef_eulers = self.convert_pose_msg_to_list(request.eef_poses)
        rob_module_name = request.robot_module_name
        
        self.control_primitives.append({
            'robot_name': rob_module_name,
            'primitive_type': 'move_eef_through_poses',
            'data': {
                'eef_pos': eef_poses,
                'eef_euler': eef_eulers
            }
        })
        
        # update robot state cache
        self.robot_modules[rob_module_name]['status'] = 'primitive_executing'
        self.robot_modules[rob_module_name]['current_eef_goal_position'] = eef_poses[-1]
        self.robot_modules[rob_module_name]['current_eef_goal_quaternion'] = eef_eulers[-1]
        
        while self.robot_modules[rob_module_name]['status'] == 'primitive_executing':
            
            # skip the first feedback
            if len(self.robot_modules[rob_module_name]['current_joint_goal_position']) == 0:
                time.sleep(0.01)
                continue
            remaining_avg_joint_angle = np.sqrt(np.sum((np.array(self.robot_modules[rob_module_name]['joint_position']) \
                                                            - np.array(self.robot_modules[rob_module_name]['current_joint_goal_position'])) ** 2))
            feedback = kube_bullet_grpc_pb2.MoveArmThroughEefPosesResponse(
                position = self.robot_modules[request.robot_module_name]['eef_pose'][0: 3],
                quaternion = self.robot_modules[request.robot_module_name]['eef_pose'][3: 7],
                remaining_avg_joint_angle = remaining_avg_joint_angle
            )
            time.sleep(0.01)
            yield feedback

    @staticmethod
    def convert_pose_msg_to_list(pose_msg):
        eef_poses = []
        eef_eulers = []
        for pose in pose_msg:
            eef_poses.append([
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ])
            eef_eulers.append([
                pose.euler.r,
                pose.euler.p,
                pose.euler.y,
            ])
        return eef_poses, eef_eulers


    ################## Gripper ################## 
    def ExecuteGripperControlPrimitive(self, request, context):

        self.control_primitives.append({
            'robot_name': request.gripper_module_name,
            'primitive_type': 'simple_gripper_control',
            'data': {
                'gripper_action': request.action_type,
                'position': request.position,
                'velocity': request.velocity,
                'force': request.force
            }
        })

        return kube_bullet_grpc_pb2.GripperStatus(
            status = 'accepted'
        )

    
    ################## Objects ##################
    def RequestObjectState(self, request, context):
        """
        Request to get object state
        """        
        obj_name = request.object_name
        if not obj_name in self.object_state:
            return kube_bullet_grpc_pb2.ObjectState(
                object_name = obj_name,
                status = 'UNKNOWN'
            )
        position = self.object_state[obj_name]['position']
        if position is None:
            return kube_bullet_grpc_pb2.ObjectState(
                object_name = obj_name,
                status = self.object_state[obj_name]['status']
            )
        else:
            return kube_bullet_grpc_pb2.ObjectState(
                object_name = obj_name,
                status = self.object_state[obj_name]['status'],
                position = self.object_state[obj_name]['position'],
                quaternion = self.object_state[obj_name]['quaternion']
            )


    ################## Debugging ##################
    def CreatePoseMarkers(self, request, context):
        """Create 3d markers for debugging purpose"""        
        cmd = {
            'resource': 'pose_marker',
            'command': 'create',
            'cmd_metadata': {
                'name': request.name,
                'parent_body': request.parent_body,
                'parent_link': request.parent_link,
                'pose_markers': request.pose_markers
            }
        }
        self.sim_setup_cmds.append(cmd)
        return kube_bullet_grpc_pb2.PoseMarkerCreateResponse(status = 'accepted')


class KubeBulletGrpcServer:
    """
    A gRPC server specifically designed to work with the Kube-Bullet simulator.
    
    It provides the interface for the simulator server to receive the commands via grpc and 
    to cache the states to be published, such as robot or object states.
    """
    
    def __init__(self, insecure_port=50051, max_workers=4) -> None:
        
        # create server with a thread pool executor
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
        
        # initialize the servicer
        self.servicer = _KubeBulletServicer(cb=self.cb_start_simulation)
        
        # add servicer and add listen port
        kube_bullet_grpc_pb2_grpc.add_KubeBulletInterfaceServicer_to_server(self.servicer, self.server)
        self.server.add_insecure_port(f"[::]:{insecure_port}")


    def start_server(self):
        """Start the server"""
        self.server.start()
        self.server.wait_for_termination()

    def start(self):
        """Start thread"""
        thread = threading.Thread(target=self.start_server)
        thread.start()
        logger.info("Started thread for gRPC server")
    
    def cb_start_simulation(self):
        """Call back -> TODO"""
        logger.info("Receiving Request to start the simulation server")
        return

    ######################### Simulation Setup #########################
    def check_new_sim_control_cmd(self):
        """
        Check whether get new simulation control command
        """
        return self.servicer.new_sim_control_cmd

    def get_simulation_control_flags(self):
        """
        Get simulation control flags
        """
        self.servicer.new_sim_control_cmd = False
        return (self.servicer.run_steps,
                self.servicer.rt_factor,
                self.servicer.run_timestep)
    
    def get_setup_command(self):
        """
        Get simulation setup command
        This method is called in every simulation spin.
        """
        return self.servicer.sim_setup_cmds
    
    def update_setup_execution_status(self,
                                      resource: str,
                                      name: List[str],
                                      status: List[str]):
        """
        Update after executing setup command
        """
        if resource == 'robot':
            for i in range(len(name)):
                self.servicer.robot_modules[name[i]].update({
                    'status': status[i]
                })
            return

        if resource == 'object':
            for i in range(len(name)):
                self.servicer.object_state[name[i]].update({
                    'status': status[i]
                })
    
    def push_back_unssucceed_commands(self, cmd):
        """
        Push back the unsuccessful setup execution.
        Possible reason is some setup need to wait for other spawning to be finished.
        """
        self.servicer.sim_setup_cmds.append(cmd)
        return


    ######################### Robots #########################
    def set_robot_state(self,
                        robot_name: str,
                        state: Optional[dict]) -> None:
        """
        cache the robot state
        """
        # print(self.servicer.robot_modules)
        self.servicer.robot_modules[robot_name]['joint_position'] = state['joint_position']
        self.servicer.robot_modules[robot_name]['status'] = state['status']
        # add eef pose for robot arm
        if 'eef_pose' in state:
            self.servicer.robot_modules[robot_name]['eef_pose'] = state['eef_pose']
        return

    def get_robot_commands(self):
        """
        Get the received robot control command
        TODO: Refactoring
        Only tested for joint_position_control
        """
        new_cmds = []
        
        for rob_name, rob_state in self.servicer.robot_modules.items():

            if rob_state not in ['spawning']:
                if rob_state['is_new_command']:
                    new_cmds.append({
                        'name': rob_name,
                        'control_cmds': {'joint_position_control': rob_state['joint_position_cmd']}
                    })
                    rob_state['is_new_command'] = False
        return new_cmds
    
    def get_primitives(self):
        """
        Get the control primitives, such as:
         - gripper control action
         - move_eef_through_poses
        """
        primitives = copy.deepcopy(self.servicer.control_primitives)
        self.servicer.control_primitives = []
        return primitives

    def update_primitive_execution_response(self,
                                            robot_module_name: str,
                                            response: dict) -> None:
        """
        Update the execution response of control primitive
        first use case: update the final goal joint position of move_eef_through_poses
        """
        if response is None:
            logger.warning(f"Module {robot_module_name} return None response for control primitive.")
            return
        # For move_eef_through_poses
        self.servicer.robot_modules[robot_module_name]['current_joint_goal_position'] = response['data']['current_joint_goal_position']
        self.servicer.robot_modules[robot_module_name]['status'] = response['status']
        return

    ######################### Objects #########################
    def set_object_state(self, 
                          object_name,
                          position,
                          quaternion
                          ) -> None:
        
        self.servicer.object_state[object_name].update({
            'position': position,
            'quaternion': quaternion,
        })
        return


if __name__ == "__main__":
    server = KubeBulletGrpcServer()
    server.start()
    
    print(threading.current_thread())
    
