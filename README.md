# Go2 safety workspace

ROS 2 Humble packages for Unitree Go2 navigation and RealSense-based safety monitoring.
The robot, localization and navigation packages are in `src/yeah_go2`.
The safety nodes and the pinned RealSense and YOLO dependencies are in `src/yeah_vision`.

```bash
git clone --recurse-submodules https://github.com/taehyun0128/capstone_ws.git yeah_ws
cd yeah_ws
```

For an existing checkout, run `git submodule update --init --recursive`.
On a machine with the robot SDK, ROS dependencies and vision runtime installed:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch yeah_bringup go2_ouster_rko_lio_nav2.launch.py \
  go2_sensor_hostname:="${OUSTER_HOST:?Set OUSTER_HOST to the sensor hostname or address}" \
  rko_global_map_path:="$PWD/local/map.ply" \
  map_yaml:="$PWD/local/map.yaml"
```

Set `OUSTER_HOST` before running the command. Supply a matching local 3D map,
occupancy YAML and occupancy image. Use `go2_udp_dest` to set the receiver address if needed.
The waypoint manager accepts `waypoints_file` and `route_name`; an example of the
file format is in `src/yeah_go2/yeah_bringup/config/waypoints.example.yaml`.

For helmet detection, set the ROS parameter `model_path` to a local weights file.
To enable audio, place your own `helmet.mp3` and `siren.mp3` in
`src/yeah_vision/my_safety_pkg/audio/` before building. For remote event-image
viewing, set `event_image_base_url` on the detection nodes to the URL reachable
by the viewer. Detection, audio and the optional web server are launched separately
from the default navigation stack.
