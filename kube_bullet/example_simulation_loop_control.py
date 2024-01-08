
import time
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

    def stop_simulation(self):
        res = self.stub.SetupSimulator(kube_bullet_grpc_pb2.SimulatorSetupRequest(
            action = 'STOP'
        ))
        print(res)
        
    def run_steps(self, steps = 100):
        res = self.stub.SetupSimulator(kube_bullet_grpc_pb2.SimulatorSetupRequest(
            action = 'RUN_STEPS',
            steps = steps
        ))
        print(res)
    
    def spawn_object(self):
        res = self.stub.SetupObject(kube_bullet_grpc_pb2.ObjectSetupRequest(
            command = 'spawn',
            object_name = 'cube_1',
            config_path = '/workspace/kube_bullet/objects/object_assets/cube/cube.bullet.config.yaml',
            position = [0.5, -0.2, 1.0],
            flags = ['SHOW_BASE_POSE']
        ))
        print(f"Object spawn status: {res} \n")

    def remove_object(self):
        res = self.stub.SetupObject(kube_bullet_grpc_pb2.ObjectSetupRequest(
            command = 'remove',
            object_name = 'cube_1',
        ))
        print(f"Object remove status: {res} \n")

    def spawn_camera(self):
        res = self.stub.SetupCamera(kube_bullet_grpc_pb2.CameraSetupRequest(
            command = 'spawn',
            camera_name = 'rs_eef_camera',
            parent_body = 'ur10e_cell__ur10e',
            parent_link = 'camera_link',
            auto_rendering = True,
            intrinsic_param = [450, 0, 320, 0, 450, 240, 0, 0, 1]
        ))

    def get_object_state(self, obj_name: str):
        res = self.stub.RequestObjectState(kube_bullet_grpc_pb2.ObjectStateRequest(
            object_name = obj_name
        ))
        print("Object state: \n")
        print(res)

    
def run():
    
    client = KubeBulletClient()

    client.stop_simulation()

    client.spawn_object()

    input('RUN')
    
    client.run_steps(100)

    # wait for running be finished
    time.sleep(100. / 240 + 0.1)
    
    client.get_object_state('cube_1')

    time.sleep(1.0)
    client.remove_object()
    
    client.get_object_state('cube_1')


if __name__ == "__main__":
    run()
