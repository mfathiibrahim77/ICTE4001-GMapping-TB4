# ICTE4001 Case Study 2: GMapping SLAM on TurtleBot4 (ROS 2 Humble)

Reproduction package for our ICTE4001 Case Study 2 report (Curtin University). We study **GMapping**, a Rao-Blackwellised particle filter SLAM algorithm, run it on a simulated TurtleBot4, and connect it to **Nav2** for autonomous navigation.

![Software architecture](figures/architecture.png)

## 1. Environment

| Item | Version / setting |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Gazebo (Ignition) Fortress, `office_world` from the unit's `turtlebot_virtual_lab` package |
| Robot | Simulated TurtleBot4 (standard model for mapping, Lite model for navigation tests) |
| SLAM | GMapping: OpenSLAM core + ROS 2 port by Project-MANAS, patched by us (see `docs/`) |
| Navigation | Nav2 (`nav2_bringup`), NavFn planner, DWB controller |
| Machine used for the runs | Curtin VDI remote desktop ("PLC - ROS2 Humble"), no GPU |

Install the ROS packages (same list as the unit's setup guide):

```bash
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot4-description ros-humble-turtlebot4-msgs ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-node ros-humble-turtlebot4-desktop ros-humble-turtlebot4-simulator \
  ros-humble-irobot-create-nodes ros-humble-teleop-twist-keyboard python3-colcon-common-extensions
```

## 2. Build

```bash
# 1. the unit's simulator workspace with the office world
cd ~/turtlebot_virtual_lab && source /opt/ros/humble/setup.bash && colcon build
source ~/turtlebot_virtual_lab/install/setup.bash

# 2. this repository
git clone https://github.com/mfathiibrahim77/ICTE4001-GMapping-TB4.git ~/ICTE4001-GMapping-TB4
cd ~/ICTE4001-GMapping-TB4 && colcon build
source install/setup.bash
```

## 3. Launch sequence

Each command in its own terminal, in this order.

```bash
# simulator, then undock
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=office_world
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}

# mapping with GMapping + RViz, drive with teleop
ros2 launch tb4_gmapping_nav gmapping.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# save the map
ros2 run nav2_map_server map_saver_cli -f ~/gmapping_map --ros-args -p save_map_timeout:=20.0
```

**Navigation, mode A: live** (GMapping provides the `map -> odom` transform, no AMCL). Keep GMapping running, then:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$(ros2 pkg prefix tb4_gmapping_nav)/share/tb4_gmapping_nav/config/nav2_params.yaml
```

**Navigation, mode B: on the saved GMapping map** (Nav2 `map_server` + AMCL, as in Practical 7). Stop GMapping first:

```bash
ros2 launch nav2_bringup localization_launch.py map:=$HOME/gmapping_map.yaml \
  params_file:=$(pwd)/config/amcl_sim.yaml use_sim_time:=true
ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$(ros2 pkg prefix tb4_gmapping_nav)/share/tb4_gmapping_nav/config/nav2_params.yaml
```

In RViz use **2D Pose Estimate** (mode B only), then **2D Goal Pose**.

## 4. Repository contents

| Path | What it is | Changed from upstream? |
| --- | --- | --- |
| `src/slam_gmapping/openslam_gmapping/` | GMapping algorithm core (OpenSLAM) | One getter added (`getWeights()`) |
| `src/slam_gmapping/slam_gmapping/` | ROS 2 GMapping node | Yes: parameters from YAML, latched `/map`, map-update timer fix, range handling, timestamp fix, entropy/N_eff, particle cloud, Humble build fixes. All marked `[ICTE4001]` |
| `src/tb4_gmapping_nav/launch/` | `gmapping.launch.py` (SLAM + RViz), `slam_nav.launch.py` (SLAM + Nav2) | Ours |
| `src/tb4_gmapping_nav/config/gmapping.yaml` | GMapping parameters tuned for the TurtleBot4 | Ours |
| `src/tb4_gmapping_nav/config/nav2_params.yaml` | Nav2 parameters (based on Practical 7) | Planner set to NavFn |
| `src/tb4_gmapping_nav/config/nav2_params_iar_astar.yaml` | Same, using our Practical 7 A* planner | Ours |
| `src/tb4_gmapping_nav/scripts/` | `slam_eval_recorder.py`, `drive_waypoints.py`, `analyse_run.py` (evaluation tools) | Ours |
| `src/tb4_cpp_prac7/` | Our Practical 7 A* global planner (Nav2 plugin) | Ours |
| `config/amcl_sim.yaml` | AMCL settings for the simulator (`base_link` instead of `base_footprint`) | Ours |
| `docs/CODE_MAP.md` | Call graph and theory-to-code table | Ours |
| `docs/ICTE4001_gmapping_changes.patch` | Full diff of our changes to the upstream port | Ours |
| `results/` | Screenshots and saved maps from our runs | Ours |

## 5. Experiments and results

| Exp. | What | Result |
| --- | --- | --- |
| E1 | Mapping: teleop through start room, corridor and second room, back to start (loop) | Clean map: straight single walls, layout matches the office world, loop closure succeeded (no doubled walls) |
| E2 | Live SLAM + Nav2 (mode A) | Nav2 planned correct paths on the live map; goals did not complete because transforms lagged (see limitations) |
| E3 | Nav2 + AMCL on the saved GMapping map (mode B) | AMCL localised the robot; Nav2 planned a path around a wall and through a doorway; goals did not complete for the same reason |

During E1 the GMapping log showed an average scan-matching score of about 610 and N_eff of about 49.8 of 50 particles. Mapping took about 40 min of real time at a simulation real-time factor below 20%.

| Figure | |
| --- | --- |
| `results/screenshots/fig1_final_map.png` | Final GMapping map |
| `results/screenshots/fig2_back_at_start.png` | Back at the start: laser aligned with the first-mapped walls (loop closure) |
| `results/screenshots/fig3_nav2_live.png` | E2: Nav2 path on the live map |
| `results/screenshots/fig4_nav2_saved_map_doorway.png` | E3: AMCL particles, costmap and path through the doorway |
| `results/screenshots/fig5_nav2_panel.png` | E3: Nav2 panel |

Video of the mapping run: *link to be added*

## 6. Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Robot jumps constantly in RViz | Two GMapping nodes running | `ros2 node list`; run only one `/slam_gmapping` |
| Robot will not move after a bump; `motion_control` warns about `back up limit` or `Reflex` | Create 3 safety reflexes | Turn away / drive forward, or restart the simulator |
| `map_saver`: `Failed to spin map subscription` | Default 2 s wait too short | `--ros-args -p save_map_timeout:=20.0` |
| AMCL: `base_footprint does not exist` | Nav2 default frame name | Use `config/amcl_sim.yaml` (`base_link`) |
| Nav2: `Transform data too old`, `Goal failed` | Computer too slow, transforms lag the sim clock | Run on a machine with a GPU; check the Gazebo real-time factor |
| Nav2 output ends with `Creating bond timer` | Normal last start-up line | Look for `Managed nodes are active` above it |
| Gazebo stuck on `Requesting list of world names` | Old Gazebo processes, or the VDI is overloaded | `pkill -f "ign gazebo"`, then relaunch or start a fresh session |

## 7. Known limitations

- Simulation only, one run per experiment.
- The VDI ran at under 20% real-time; `map -> odom` arrived 20–140 s late, so Nav2's controller rejected the plans. This also happened with AMCL, so it is a computing limit rather than a GMapping fault.
- The 2D laser cannot see the charging dock (it is below the scan plane).
- GMapping has no explicit loop-closure module; it relies on good particles surviving resampling, which can fail on long loops in large environments.
- Wheel slip after collisions caused larger pose corrections.

## 8. Notes

- GMapping core: G. Grisetti, C. Stachniss and W. Burgard (OpenSLAM, BSD licence). ROS 2 port: Project-MANAS (https://github.com/Project-MANAS/slam_gmapping).

## Authors

ICTE4001 Group ___: _names / student IDs_
