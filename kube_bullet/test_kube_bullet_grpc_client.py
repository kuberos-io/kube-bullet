

import grpc

from grpc_kube_bullet import kube_bullet_grpc_pb2
from grpc_kube_bullet import kube_bullet_grpc_pb2_grpc



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
        res = self.stub.SpawnRobot(kube_bullet_grpc_pb2.RobotMetadata(
            robot_name = 'ur10e',
            active_components = ['arm_ur10e', 'robotiq_gripper']
        ))
        print(f"Robot spawn status: {res}")

    def spawn_object(self):
        res = self.stub.SpawnObject(kube_bullet_grpc_pb2.ObjectMetadata(
            object_name = 'cube_1',
        ))
        print(f"Object spawn status: {res}")

    def send_joint_position(self):
        robot_joint_position_command = kube_bullet_grpc_pb2.RobotJointPositionCommand(
            robot_name = 'ur10e',
            component_name = 'arm_ur10e',
            positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
        
        request = kube_bullet_grpc_pb2.MoveArmThroughEefPosesRequests(
            robot_name='ur10e__arm_ur10e',
            eef_link_name='tcp_link',
            eef_poses=[pose_euler_pre, pose_euler]
        )
        print("Request: move eef to pose: ")
        print(pose_euler)
        
        res = self.stub.MoveArmThroughEefPoses(request)
        print(res)
        
    def close_gripper(self):
        gripper_control_command = kube_bullet_grpc_pb2.GripperControlCommand(
            robot_name='ur10e',
            component_name='robotiq_gripper',
            action_type='close',
        )
        
        print("Request: close the gripper")
        
        res = self.stub.SendGripperControlCommand(gripper_control_command)
        print(res)
    
    def pick_up_object(self):
        pos = kube_bullet_grpc_pb2.pos(x=0.5, y=-0.2, z=1.2)
        rpy = kube_bullet_grpc_pb2.rpy(r=0, p=0, y=0)

        pose_euler = kube_bullet_grpc_pb2.pose_euler(
            position=pos,
            euler=rpy
        )
        
        request = kube_bullet_grpc_pb2.MoveArmThroughEefPosesRequests(
            robot_name='ur10e__arm_ur10e',
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
    
    input("Move EEF?")
    # client.send_joint_position()
    client.move_eef_through_poses()
    
    input("Closing gripper?")
    client.close_gripper()

    input("Pick up?")
    client.pick_up_object()


if __name__ == "__main__":
    run()
