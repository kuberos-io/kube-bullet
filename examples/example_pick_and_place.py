
import grpc

from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2
from kube_bullet.grpc_kube_bullet import kube_bullet_grpc_pb2_grpc


class KubeBulletClient:
    
    def __init__(self,
                 insecure_port: int=50051) -> None:
        
        self._insecure_port = insecure_port
        
        self.channel = grpc.insecure_channel(f"0.0.0.0:{self._insecure_port}")
        self.stub = kube_bullet_grpc_pb2_grpc.KubeBulletInterfaceStub(self.channel)
        
        
    def __delete__(self):
        super(self).__delete__()
        self.channel.close()    

    def spawn_robot(self):
        res = self.stub.SetupRobot(kube_bullet_grpc_pb2.RobotSetupRequest(
            command = 'spawn',
            robot_name = 'ur10e_cell',
            active_components = ['ur10e', 'robotiq_gripper'],
            robot_config_path="/workspace/kube_bullet/robots/robot_assets/ur10e_cell/ur10e_cell.bullet.config.yaml"
        ))
        print(f"Robot spawn status: {res}")

    def spawn_object(self):
        res = self.stub.SetupObject(kube_bullet_grpc_pb2.ObjectSetupRequest(
            command = 'spawn',
            object_name = 'cube_1',
            config_path = '/workspace/kube_bullet/objects/object_assets/cube/cube.bullet.config.yaml',
            position = [0.5, -0.2, 1.0],
            flags = ['SHOW_BASE_POSE']
        ))
        print(f"Object spawn status: {res}")

    def spawn_camera(self):
        res = self.stub.SetupCamera(kube_bullet_grpc_pb2.CameraSetupRequest(
            command = 'spawn',
            camera_name = 'rs_eef_camera',
            parent_body = 'ur10e_cell__ur10e',
            parent_link = 'camera_link',
            auto_rendering = True,
            intrinsic_param = [450, 0, 320, 0, 450, 240, 0, 0, 1]
        ))
    
    def send_joint_position(self):
        robot_joint_position_command = kube_bullet_grpc_pb2.RobotJointPositionCommand(
            robot_module_name = 'ur10e_cell__ur10e',
            positions = [0 - 0.3, -1.57079632, 1.57079632, -1.57079632, -1.57079632, 0 - 0.3]
        )
        print(robot_joint_position_command)
        res = self.stub.SendRobotJointPositionCommand(robot_joint_position_command)
        print(res)
    
    def move_eef_through_poses(self):
        pos_pre = kube_bullet_grpc_pb2.pos(x=0.5, y=-0.2, z=1.15)
        pos = kube_bullet_grpc_pb2.pos(x=0.5, y=-0.2, z=1.0)
        rpy = kube_bullet_grpc_pb2.rpy(r=0, p=0, y=0)
        
        pose_euler_pre = kube_bullet_grpc_pb2.pose_euler(
            position=pos_pre,
            euler=rpy
        )
        pose_euler = kube_bullet_grpc_pb2.pose_euler(
            position=pos,
            euler=rpy
        )
        
        request = kube_bullet_grpc_pb2.MoveArmThroughEefPosesRequest(
            robot_module_name='ur10e_cell__ur10e',
            eef_link_name='tcp_link',
            eef_poses=[pose_euler_pre, pose_euler]
        )
        print("Request: move eef to pose: ")
        print(pose_euler)
        
        res = self.stub.MoveArmThroughEefPoses(request)
        print(res)
        
    def close_gripper(self):
        gripper_control_primitive = kube_bullet_grpc_pb2.GripperControlPrimitive(
            gripper_module_name='ur10e_cell__robotiq_gripper',
            action_type='close',
        )
        
        print("Request: close the gripper")
        
        res = self.stub.ExecuteGripperControlPrimitive(gripper_control_primitive)
        print(res)
    
    def pick_up_object(self):
        pos = kube_bullet_grpc_pb2.pos(x=0.5, y=-0.2, z=1.2)
        rpy = kube_bullet_grpc_pb2.rpy(r=0, p=0, y=0)

        pose_euler = kube_bullet_grpc_pb2.pose_euler(
            position=pos,
            euler=rpy
        )
        
        request = kube_bullet_grpc_pb2.MoveArmThroughEefPosesRequest(
            robot_module_name='ur10e_cell__ur10e',
            eef_link_name='tcp_link',
            eef_poses=[pose_euler]
        )
        print("Request: move eef to pose: ")
        print(pose_euler)
        
        res = self.stub.MoveArmThroughEefPoses(request)
        print(res)

        
def run():
    
    client = KubeBulletClient()
    
    
    
    client.spawn_robot()

    client.spawn_object()
    
    client.spawn_camera()
    
    input("Send new joint position?")
    client.send_joint_position()
    
    input("Move EEF?")
    client.move_eef_through_poses()
    
    input("Closing gripper?")
    client.close_gripper()

    input("Pick up?")
    client.pick_up_object()


if __name__ == "__main__":
    run()
