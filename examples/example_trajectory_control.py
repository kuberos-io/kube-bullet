
"""
Example to demonstrate how to use the grpc interface to control 
the robot with an externally generated trajectory.
"""
import time
from loguru import logger
import numpy as np
import grpc

from grpc_kube_bullet import kube_bullet_grpc_pb2
from grpc_kube_bullet import kube_bullet_grpc_pb2_grpc

from utils.tf_utils import T

# external libraries
import py_totg
from tracikpy import TracIKSolver



class KubeBulletClient:
    
    def __init__(self,
                 insecure_port: int=50051) -> None:
        
        self._insecure_port = insecure_port
        
        self.setup()
        
    def setup(self):
        self.channel = grpc.insecure_channel(f"0.0.0.0:{self._insecure_port}")
        self.stub = kube_bullet_grpc_pb2_grpc.KubeBulletInterfaceStub(self.channel)

    def __delete__(self):
        super(self).__delete__()
        self.channel.close()    

    def ping(self):
        res = self.stub.CheckServerConnection(
            kube_bullet_grpc_pb2.Ping(ping=True)
        )
        print("Check server connection: ", res)

    def spawn_robot(self):
        res = self.stub.SetupRobot(kube_bullet_grpc_pb2.RobotSetupRequest(
            command = 'spawn',
            robot_name = 'ur10e_cell',
            active_components = ['ur10e', 'robotiq_gripper'],
            robot_config_path="/workspace/kube_bullet/robots/robot_assets/ur10e_cell/ur10e_cell.bullet.config.yaml"
        ))
        print(f"Robot spawn status: {res}")

    def spawn_object(self):
        res = self.stub.SpawnObject(kube_bullet_grpc_pb2.ObjectMetadata(
            object_name = 'cube_1',
        ))
        print(f"Object spawn status: {res}")

    def spawn_camera(self):
        res = self.stub.SpawnCamera(kube_bullet_grpc_pb2.CameraSpawnRequest(
            camera_name = 'rs_eef_camera',
            attach_body_name = 'ur10e_cell__ur10e',
            attach_link_name = 'camera_link',
            auto_rendering = True,
            intrinsic_param = [450, 0, 320, 0, 450, 240, 0, 0, 1]
        ))
    
    def send_joint_position(self, positions):
        robot_joint_position_command = kube_bullet_grpc_pb2.RobotJointPositionCommand(
            robot_name = 'ur10e_cell__ur10e',
            component_name = 'arm_ur10e',
            positions = positions
            # positions = [0 - 0.3, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0 - 0.3]
        )
        # print(robot_joint_position_command)
        res = self.stub.SendRobotJointPositionCommand(robot_joint_position_command)
        # print(res)

    def create_markers(self, poses):
        pose_markers = []
        for pose in poses:
            pose_markers.append(kube_bullet_grpc_pb2.PoseMarker(
                name=pose.name,
                position = pose.translation,
                quaternion = pose.quaternion,
            ))
        
        create_request = kube_bullet_grpc_pb2.PoseMarkerCreateRequest(
            name='predicted_eef_pose',
            parent_body='ur10e_cell__ur10e',
            parent_link='base_link_inertia',
            pose_markers=pose_markers
        )
        
        res = self.stub.CreatePoseMarkers(create_request)
        
        print(res)


class TrajectoryGenerator:
    
    UR10E_MAX_VEL = np.array([2.09, 2.09, 3.14, 3.14, 3.14, 3.14]) / 2 # Data from ur10e datasheet
    UR10E_MAX_ACC = np.array([3, 3, 2, 1, 1, 1]) * 2 # CHECK: No public data
    UR10E_MAX_TORQUE = np.array([330, 330, 150, 56, 56, 56]) # Data from ur10e datasheet
    
    JOINT_LOWER_LIMIT = np.pi * np.array([-2, -2, -1, -2, -2, -2])
    JOINT_UPPER_LIMIT = np.pi * np.array([2, 2, 1, 2, 2, 2])
    
    IK_INIT_POSITION = [0, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0]
    
    def __init__(self, control_freq=240) -> None:
        self.trajectory_generator = py_totg.ToTg(6, 
                                                 1.0 / control_freq,
                                                 self.UR10E_MAX_VEL,
                                                 self.UR10E_MAX_ACC)
        self.max_deviation = 0.005

        self.join_trajecotry = None
        
        self.ik_solver = TracIKSolver(
            "robots/robot_assets/ur10e_cell/ur10e_only.urdf",
            "base_link_inertia", "wrist_3_link",
            timeout=0.05
        )
        
        print("Joint limits from URDF: \n", self.ik_solver.joint_limits)
        print("Change joint limits")
        self.ik_solver.joint_limits = [self.JOINT_LOWER_LIMIT.tolist(),
                                       self.JOINT_UPPER_LIMIT.tolist()]
        
        
    def get_trajectory_pos(self, waypoints):
        
        self.trajectory_generator.set_waypoints(waypoints,
                                                self.max_deviation)
        
        self.trajectory_generator.compute_trajectory()
        
        res_pos = np.array(self.trajectory_generator.get_res_position())
        
        return res_pos
        
    def generate_trajectory_from_eef(self, eef_poses):
        
        waypoints = []
        for pose in eef_poses:
            j_position = self.ik_solver.ik(pose.matrix,
                                           qinit=self.IK_INIT_POSITION)
            waypoints.append(j_position)
            print(j_position)

        self.trajectory_generator.set_waypoints(waypoints, self.max_deviation)
        self.trajectory_generator.compute_trajectory()
        
        return np.array(self.trajectory_generator.get_res_position())


init_pose = T(translation=(-0.6914, -0.17415, 0.67685),
              quaternion=(0.7071, 0.7071, 0.0, 0.0),
              name='initial_pose')

sec_pose = T(translation=(-0.6, -0.1, 0.60685),
              quaternion=(0.7071, 0.7071, 0.0, 0.0),
              name='pose 2')

third_pose = T(translation=(-0.7, 0.0, 0.60685),
              quaternion=(0.7071, 0.7071, 0.0, 0.0),
              name='pose 3')

pre_pose = T(translation=(-0.945, 0.2, 0.35),
              quaternion=(0.7071, 0.7071, 0.0, 0.0),
              name='pre_grasp_pose')

final_pose = T(translation=(-0.945, 0.2, 0.22),
              quaternion=(0.7071, 0.7071, 0.0, 0.0),
              name='grasp_pose')


def run():
    
    client = KubeBulletClient()
    
    num_hand_check = 10
    while num_hand_check > 0:
        try:
            client.ping()
            break
        except grpc._channel._InactiveRpcError:
            print("The RPC failed because the channel is inactive.")
            num_hand_check -= 1
            client.channel.close()
            time.sleep(0.5)
            client.setup()

    client.spawn_robot()

    client.spawn_object()
    
    time.sleep(3)
    
    # client.spawn_camera()
    
    input("Move for grasping")
    
    poses = [init_pose, sec_pose, third_pose, pre_pose, final_pose]
    
    client.create_markers(poses)
    
    control_freq = 240
    
    waypoints = [
        [0 - 0.3, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0 - 0.3],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0 + 0.3, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0 + 0.3]
    ]
    
    tra_gen = TrajectoryGenerator(control_freq)
    
    # trajetory = tra_gen.get_trajectory_pos(waypoints)
    trajetory = tra_gen.generate_trajectory_from_eef(poses)

    print("Trajectory length: ", trajetory.shape[0])
    
    for i in range(trajetory.shape[0]):
        
        start = time.time()
        
        # send trajectory
        client.send_joint_position(list(trajetory[i, :]))
        
        end = time.time()
        
        sleep_time = 1 / control_freq - (end - start)
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            logger.warning(f"Execution exceeds expected loop time: {(end - start) * 1000 } ms")


if __name__ == "__main__":
    run()
