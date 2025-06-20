# amiga-ros2-nav

## Installation

1. Install vcs following the instructions [here](https://github.com/dirk-thomas/vcstool?tab=readme-ov-file#how-to-install-vcstool)

2. Run `vcs import < nav.repos` to install the required 3rd party packages

3. Build and Start the docker container with `make build-prod` and `make bash`

4. Install dependencies
```bash
rosdep update; rosdep install --from-paths . --ignore-src -r -y
```

5. Run `colcon build`

## Foxglove

1. Inside the docker container, install the ros2 foxglove node with 
```bash
apt install ros-humble-foxglove-bridge
```

2. Run the foxglove node (Farm-ng uses port `8765` for their foxglove bridge, so use `8766`)
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="8766"
```

3. Follow instructions [here](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2) to open foxglove

## Running

1. TMUX Cheatsheet
Open tmux: `tmux new -s ros`
Attach tmux: `tmux attach -t ros`
Kill tmux: `tmux kill-session -t ros`
ctrl-b +
    c : new pane
    [number] : switch pane
    n : next window
    p : previous window
    w : list windows & select
    `[` : scroll mode (press q to escape)
    d : exit tmux
    x : close pane


1. Bringup the ros2_bridge, twist_control, joy, urdf, localization
```bash
make foxglove
make amiga-streams
make twist
make description
make oakd
make nav
make joy
```

imu1.urdf.xacro
<!-- <origin xyz="-0.26 -0.28 0.77" rpy="3.14159 1.5708 3.14159"/> -->