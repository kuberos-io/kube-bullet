# Kube-Bullet
A Pybullet-based robotics simulation using gRPC for client-server communication.


## Development Environment

### For VSCode users (recommended):
You may need to adapt the [docker-compose.yaml](.devcontainer/docker-compose.yml) depending on you system setup.

 - devcontainer image: By default, we recommend using the pre-built devcontainer image. If you want to install additional packages, please modify the [Dockerfile](.devcontainer/Dockerfile) and change the dockerfile arg in [docker-compose.yaml](.devcontainer/docker-compose.yml).

 - nvidia gpu support: If you want to use the gpu acceleration, please comment out the args with `nvidia` in [docker-compose.yaml](.devcontainer/docker-compose.yml)

### Using VIM or other IDE:
Clone the code and stay in the project root folder `kube-bullet` and start the container with 
```bash
kube-bullet$ ./containers/devcontainer_without_vscode/run_nvidia.sh
# If you are familiar with Docker, you can run this daemon container in detached mode.
```
To get more terminal inside container:
```bash
kube-bullet$ ./containers/devcontainer_without_vscode/exec.sh
```

The code in the project folder are directly mounted into the `/workspace` in `DevContainer`.


## Quick Start

Install this project if you do not use VSCode.
```bash
pip install -e
```

Note: You can also use it directly in your python virtual environment. The `trajectory_control` example requires two additional libraries for ik and trajectory generation, see [`DockerFile`](.devcontainer/Dockerfile) for details. For other dependencies, see [`requirements`](requirements.txt).

Start the simulation instance in the first terminal with:
```bash
python -m kube_bullet.run_simulation
```

Run example in a second terminal:
```bash
python examples/example_pick_and_place.py
# Press ENTER to perform the next steps.
```

More examples, see: [examples](examples/)
