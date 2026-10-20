# Practical 7 – A* planner: how to run it in the Virtual Lab and how the code works

## 1. Run it in the Virtual Lab (simulated TurtleBot4)

Changes made for the simulator: the simulated office map is included in `map/` (installed
with the package). Nothing else in the configuration needs to change; you only pass
`use_sim_time:=true` on the command line (the launch file defaults to `false` for the real robot).

```bash
# build (put tb4_cpp_prac7 in your workspace's src/ folder first)
cd ~/prac_ws && colcon build --packages-select tb4_cpp_prac7 && source install/setup.bash

# T1: simulator (TurtleBot4 in the office world, without its own SLAM/Nav2)
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=false localization:=false

# T2: localisation (AMCL + map_server) on the SIMULATED office map
ros2 launch nav2_bringup localization_launch.py use_sim_time:=true \
     map:=$(ros2 pkg prefix tb4_cpp_prac7)/share/tb4_cpp_prac7/map/office_ICTE4001_sim.yaml

# T3: RViz
ros2 launch turtlebot4_viz view_robot.launch.py

# T4: undock
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}

# T5: Nav2 with YOUR A* planner
ros2 launch tb4_cpp_prac7 navigation_launch.py use_sim_time:=true
```
In RViz: **2D Pose Estimate** (where the robot really is), then **2D Goal Pose**.
A green path appears and the robot drives along it.

To compare with the default planner, stop T5 and run
`ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`, send the same goal.

**If the robot doesn't move:** check `use_sim_time:=true` on every command, that you set the
initial pose, and that the map lines up with the laser scan in RViz.

---

## 2. What the code does (be ready to explain every line)

**Big idea.** The costmap is a grid. Each cell has a *cost* (`costarr_`: 50 = free floor,
up to 253 near walls, 254 = wall). The planner computes for each cell a *potential*
(`potarr_`) = the cheapest total cost to get there **from the start**. It spreads outwards
from the start like a wave ("wavefront"). Then the path is found by walking back from the
goal, always stepping to the neighbour with the lowest potential, until reaching the start.

**What makes it A\*.** Plain Dijkstra spreads the wave equally in all directions. A\* sorts
cells by `f = g + h`:
* `g` = potential, the real cost from the start to this cell (`potarr_[n]`)
* `h` = heuristic, the straight-line distance from this cell to the goal × `COST_NEUTRAL`
So cells **towards the goal** are expanded first and far fewer cells are visited.
`h` uses `COST_NEUTRAL` (50, the cheapest cost of one step) so it never over-estimates the
real remaining cost → the result stays (near-)optimal.

**Three priority buffers instead of a sorted queue** (fast, approximate priority queue):
* `curPotentArr_` – cells being processed in this cycle
* `nextPotentArr_` – high priority (f below the threshold `potentThresh_`)
* `overflowPotentArr_` – low priority (f above the threshold)
* `pending_[n]` – cell n is already waiting in a buffer (don't add it twice)

### Milestone by milestone

| # | Code | Why |
|---|---|---|
| 1.1 | push the 4 **diagonal** neighbours of the start too | the grid is 8-connected (updateCell uses 8 neighbours), so the wave must start in all 8 directions |
| 2.1 | `dist = hypot(goal − start) · COST_NEUTRAL; potentThresh_ += dist` | every cell's priority f = g + h is at least h(start) = dist, so the threshold must start above it; otherwise everything would go to overflow |
| 2.2 | `if (n_cur == 0 && n_next == 0) break` | no cells left to expand → stop (goal unreachable) |
| 2.3 | `pending_[cell] = false` for all cells in `curPotentArr_` | they are now being processed, so they may be pushed again later if a cheaper route is found. Also declares `pb` (pointer) and `i` (counter) used below |
| 2.4 | if `curPotentArr_` is empty after the swap: swap it with overflow, `potentThresh_ += potentThreshInc_` | all high-priority cells done → promote the low-priority ones and raise the bar |
| 2.5 | if `potarr_[goal] < POT_HIGHEST` → success, break | the goal got a potential → the wave reached it → a path exists. Goal index = `goal_[1] * nx_ + goal_[0]` (row × width + column) |
| 3.1 | `l = potarr_[n−1] + costarr_[n]` etc. (`r`: n+1, `t`: n−nx_, `b`: n+nx_) | cost to reach n via that neighbour = neighbour's potential + cost of stepping into n. `nx_` = map width, so ±nx_ is one row up/down |
| 3.2 | diagonals with `√2 · costarr_[n]` | a diagonal step is √2 times longer |
| 3.3 | `pot = std::min({l, r, t, b, tl, tr, bl, br})` | cheapest way into n |
| 3.4 | `potarr_[n] = pot` | store g(n) (only done if it is lower than before) |
| 3.5 | `x = n % nx_`, `y = n / nx_`, `pot += hypot(goal − (x,y)) · COST_NEUTRAL` | convert index → (column, row); pot becomes f = g + h. Note potarr_ keeps g only |
| 3.6/3.7 | f < threshold → push neighbours that could improve to `nextPotentArr_` | high priority. Condition: neighbour's potential > pot + cost of that neighbour (×√2 for diagonals) |
| 3.8/3.9 | same check, but push to `overflowPotentArr_` | f ≥ threshold → low priority, processed later |
| 4.1 | check the 8 neighbours of `stc`, keep the one with the smallest potential, `stc = minc` | walk "downhill" from goal to start. Stops when potential < COST_NEUTRAL (= the start cell, potential 0) |

Useful facts the tutor may ask:
* **Index ↔ coordinates**: `n = y·nx_ + x`, `x = n % nx_`, `y = n / nx_`.
* **Why the border is set to COST_OBS**: so neighbour indexes `n±1`, `n±nx_` never leave the map.
* **Why `pending_`**: stops the same cell being added to a buffer many times.
* **Why unknown cells cost 253**: `allow_unknown` lets the planner go through unexplored space, but expensive.
* **calcPath returns the path from goal to start**; `iar_astar_planner.cpp` turns cell coordinates into
  map coordinates and reverses it for Nav2.

---

## 3. "Why is my path different from the default Nav2 planner?" (you must explain this)

Default `nav2_navfn_planner` (in `nav2_bringup`) uses **Dijkstra** (`use_astar: false`),
propagates from the goal over a **4-connected** grid with a **quadratic interpolation** of the
potential, and extracts the path by **gradient descent** on the interpolated potential field.
Our planner:

1. **8-connected grid, min over neighbours, discrete path extraction** → the path is made of
   straight segments and 45° diagonals (angular, "zig-zag" corners); the default path can head
   in any direction, so it is **smoother** and cuts corners more naturally.
2. **A\* heuristic** → explores far fewer cells (faster), wave stretched towards the goal.
3. **Bucket priority (threshold) + stop as soon as the goal is reached** → the path is
   **near-optimal**, not guaranteed optimal (in tests: within 0.5% of the optimal cost).
4. Both use the same costmap cost (inflation), so both keep away from walls similarly.

---

## 4. Verification done

`navfn.cpp` was compiled outside ROS and run on the costmap of `office_ICTE4001_sim`
(walls + 0.55 m inflation): 4 start/goal pairs, all paths found, collision-free, connected,
and within 0–0.5% of the optimal cost computed by an exact Dijkstra.
Not yet run inside ROS 2 / Gazebo: do Section 1 before your assessment.
