# GMapping code map (for the "Implementation Analysis" section)

Line numbers refer to the files in `src/slam_gmapping/` **after** our patch.
Paths: `node/` = `slam_gmapping/src/slam_gmapping.cpp`, `gsp/` = `openslam_gmapping/`.
Use this to guide your own reading of the code (VS Code "Go to Definition") and draw the
architecture diagram in your own words.

## 1. Two layers

| Layer | Package | Role |
|---|---|---|
| ROS 2 wrapper | `slam_gmapping` (node `slam_gmapping`) | ROS I/O: subscribes to `/scan` + TF `odom→base_link`, converts to GMapping types, publishes `/map`, TF `map→odom`, diagnostics |
| Algorithm core | `openslam_gmapping` (plain C++ libs: `gridfastslam`, `scanmatcher`, `sensor_*`, `utils`) | RBPF: motion model, scan matcher, weights, resampling, trajectory tree, per-particle maps |

## 2. Call graph from `main()`

```
main()                                                        node/:615
└─ SlamGmapping::SlamGmapping()   (rclcpp::Node "slam_gmapping")   node/:35
   ├─ tf2_ros::Buffer + TransformListener + TransformBroadcaster
   ├─ init()            declare ROS 2 params, new GridSlamProcessor    node/:56
   └─ startLiveSlam()                                                 node/:130
      ├─ publishers: /map, /map_metadata (latched), /entropy, ~/neff, ~/particles
      ├─ message_filters::Subscriber</scan>  →  tf2_ros::MessageFilter(target = odom)
      │      (a scan is delivered only once TF odom→laser exists at the scan's stamp)
      │      └─ callback: laserCallback()
      └─ std::thread publishLoop()  → publishTransform() @ 1/transform_publish_period Hz
                                       (broadcasts the latest map→odom)

laserCallback(scan)                                                    node/:404
├─ [first scan] initMapper(scan)                                       node/:201
│   ├─ laser pose wrt base_link via TF; checks the laser is planar (upright / upside down)
│   ├─ builds centered laser angles, maxRange/maxUrange
│   ├─ new RangeSensor("FLASER"), OdometrySensor
│   ├─ gsp->setMatchingParameters(maxUrange, maxRange, sigma, kernelSize, lstep, astep,
│   │                             iterations, lsigma, ogain, lskip)
│   ├─ gsp->setMotionModelParameters(srr, srt, str, stt)
│   ├─ gsp->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold)
│   └─ gsp->init(particles, xmin, ymin, xmax, ymax, delta, initialPose = odom pose)
├─ addScan(scan, odom_pose)                                            node/:352
│   ├─ getOdomPose(): TF odom→(centered laser) at scan stamp  = u_t
│   ├─ ranges: < range_min or ±inf → range_max ; reverse if angle_increment < 0
│   └─ gsp->processScan(RangeReading{ranges, pose = odom})  ─────────────► §3
├─ best = particles[getBestParticleIndex()]   (max accumulated log-weight)  gsp/gridfastslam/gridslamprocessor.cpp:498
├─ map_to_odom = (odom→laser · (laser→map of best particle))^-1   → used by publishTransform()
├─ publishParticles(), publish /entropy and ~/neff
└─ if map_update_interval elapsed: updateMap(scan)                     node/:490
    ├─ new ScanMatcherMap; for each node n of the BEST particle's trajectory tree:
    │      matcher.registerScan(map, n->pose, n->reading)   (re-render from scratch)
    └─ threshold cell occupancy: <0 → -1 (unknown), > occ_thresh → 100, else 0 → publish /map
```

## 3. One filter update: `GridSlamProcessor::processScan`  (gsp/gridfastslam/gridslamprocessor.cpp:316)

```
for each particle i:                                    PREDICTION (motion model)
    x_i ← MotionModel::drawFromMotion(x_i, odom_t, odom_{t-1})      gsp/gridfastslam/motionmodel.cpp:25
accumulate travelled distance / rotation
if  moved ≥ linearUpdate  or  rotated ≥ angularUpdate  or  (temporalUpdate ≥ 0 and time elapsed):
    scanMatch(z_t)                                       gsp/include/gmapping/gridfastslam/gridslamprocessor.hxx:9
        for each particle i:
            x* ← ScanMatcher::optimize(map_i, x_i, z_t)           gsp/scanmatcher/scanmatcher.cpp:337
            if score(x*) > minimumScore:  x_i ← x*                (else keep odometry sample)
            l ← likelihoodAndScore(map_i, x_i, z_t)               gsp/include/gmapping/scanmatcher/scanmatcher.h:191
            w_i ← w_i + l ;  weightSum_i ← weightSum_i + l        (LOG weights)
            computeActiveArea(map_i, x_i, z_t)
    updateTreeWeights() → normalize(): w~_i = exp((w_i - max w)/(ogain·N)) / Σ ;  N_eff = 1/Σ w~_i²
    resample(z_t)                                        gridslamprocessor.hxx:70
        if N_eff < resampleThreshold · N:  uniform (low-variance) resampling of indexes
        each surviving particle gets a new TNode (pose, reading) in the trajectory tree
        (particles share map memory: copy-on-write of the "active area" only)
        for each particle: registerScan(map_i, x_i, z_t)        MAPPING per particle
    return true
else: return false   (prediction only, no correction, no map update)
```

## 4. Theory ↔ code (what matches and what differs from the papers)

| Theory (Grisetti et al. 2005/2007) | Code | Note for the report |
|---|---|---|
| Factorisation p(x₁:t, m \| z, u) = p(m \| x, z) · p(x \| z, u) | each `Particle` = pose + own `ScanMatcherMap` + node in trajectory tree | Rao-Blackwellisation: map is computed analytically per trajectory |
| Motion model p(x_t \| x_{t-1}, u_t) | `drawFromMotion(p, pnew, pold)`: Gaussian noise on (dx, dy, dθ) with σ = srr·\|dx\| + str·\|dθ\| + 0.3·srr·\|dy\|, σ_θ = stt·\|dθ\| + srt·√(dx²+dy²) | Same role as α₁…α₄ in Thrun's odometry model (Prac 5/6 AMCL) |
| Improved proposal: sample from N(μ, Σ) around the scan-match mode, weight ∝ η = Σ_j p(z\|m,x_j)p(x_j\|x,u) | `scanMatch()`: particle is **moved to** the hill-climbing optimum; weight += log-likelihood at that pose. `ScanMatcher::likelihood()` (which samples with `llsamplerange` etc.) is **never called** | Implementation uses the *mode* of the proposal, not samples from it; the four `*sample*` parameters have no effect |
| Scan matching | `optimize()`: greedy hill climbing over 6 moves (±x, ±y, ±θ), step halved on no improvement, `iterations` refinements, start lstep/astep | Score s = Σ_beams exp(−\|μ\|²/σ), μ = beam end-point minus mean hit point of the best cell in a (2·kernelSize+1)² window whose "free" neighbour along the beam is free (avoids matching the back of walls) |
| Observation likelihood p(z_t \| m, x_t) | `likelihoodAndScore()`: l = Σ_beams −\|μ\|²/lsigma (or a constant `noHit` if no cell found) | A Gaussian end-point (likelihood-field-like) model; beams > maxUrange skipped |
| Adaptive (selective) resampling, N_eff | `normalize()` + `resample()`: resample only if N_eff < resampleThreshold·N | `ogain` flattens the weights (gain 1/(ogain·N)) to fight particle depletion |
| Occupancy grid mapping | `registerScan()`: Bresenham-like ray: free cells `update(false)`, end cell `update(true, hit point)`; cell occupancy = n_hits / n_visits (`PointAccumulator`) | A **counting** model (not log-odds); each cell also stores the mean hit position → sub-cell accuracy |
| Loop closure | no explicit detector / pose graph. Implicit: particles whose map agrees with the revisited area get high weight and survive resampling; the published map is re-rendered along the best particle's whole trajectory | Works only if a "correct" hypothesis survives until the loop closes → needs enough particles; long loops can fail |
| Best estimate | `getBestParticleIndex()`: max **accumulated** log-weight `weightSum` | map→odom TF and /map come from this particle |

## 5. ROS 2 interface (inputs / outputs)

| Direction | Name | Type | Notes |
|---|---|---|---|
| in | `/scan` | sensor_msgs/LaserScan | SensorDataQoS (best effort) |
| in | TF `odom → base_link → laser` | tf2 | odometry u_t + laser mounting |
| out | `/map` | nav_msgs/OccupancyGrid | reliable + transient_local (patched) |
| out | `/map_metadata` | nav_msgs/MapMetaData | |
| out | TF `map → odom` | tf2 | drift correction, 20 Hz |
| out | `/entropy` | std_msgs/Float64 | entropy of normalised weights (fixed) |
| out | `/slam_gmapping/neff` | std_msgs/Float64 | N_eff (added) |
| out | `/slam_gmapping/particles` | geometry_msgs/PoseArray | particle poses (added) |

## 6. Our modifications to the upstream ROS 2 port (Project-MANAS/slam_gmapping @3c3de50)

Full diff: `ICTE4001_gmapping_changes.patch`.

1. **Parameters**: upstream hard-coded every value; now declared as ROS 2 parameters (ROS 1 names).
2. **Map QoS**: `/map` latched (transient_local); Nav2 `static_layer` and `map_saver` need this.
3. **Map update timer bug**: `last_map_update` was a local variable → map rebuilt from the whole trajectory tree on *every* processed scan (CPU grows with run length). Now a member.
4. **maxRange/maxUrange** were overwritten with the scan's `range_max`; now configurable.
5. **±inf ranges** (no return) mapped to `range_max` so they can clear free space.
6. **Reading timestamp** used whole seconds only (`stamp.sec`) → `temporalUpdate` quantised to 1 s.
7. **Entropy** computed from log-weights (meaningless); now from normalised weights. Added N_eff.
8. **tf_delay** was cast to whole seconds (always 0).
9. **Build on Humble**: C++17 (rclcpp requirement), removed `-fuse-ld=lld`, `executable=` in launch.
10. **Particle cloud** publisher for RViz.
