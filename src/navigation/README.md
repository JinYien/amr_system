# navigation

## Overview

This package makes the robot drive somewhere on its own. One launch file runs
the Nav2 navigation stack, which keeps track of where the robot is on a saved
map, plans a path to the goal, follows that path and recovers when it gets
stuck. A second launch file runs the mapping tool that creates the saved map
in the first place. Goals come from the webpage or from RViz, either as one
goal pose or as a list of waypoints to pass through.

## Algorithms

- **Localization (AMCL).** A cloud of pose guesses (particles) is moved with
  the odometry and weighted by how well the laser scan fits the map. It is
  tuned to trust the accurate laser-inertial odometry strongly, because on a
  route where every corridor looks the same a loose tuning could jump to a
  look-alike corridor.
- **Costmaps.** The map plus the live scan are turned into a grid of driving
  costs. Around every wall there is a forbidden band the size of the robot
  half-width. The global costmap adds a wide cost slope so the cheapest line
  runs down the corridor centre. The local costmap keeps no slope at all so
  the narrowest gaps stay drivable.
- **Global planning (NavFn).** The cheapest path to the goal is searched on
  the global costmap and refreshed twice per second, which is also how the
  robot routes around obstacles that appear in the scan.
- **Path following (Regulated Pure Pursuit).** The controller chases a point
  a little ahead on the path, slows down in curves, and stops to rotate on the
  spot when the path turns sharply, as at the end of a corridor.
- **Recovery behaviors.** When progress stalls, the stack clears its costmaps,
  backs up a little and waits briefly, then tries again.
- **Waypoint navigation.** With a list of poses, the planner plans through all
  of them and each waypoint is dropped once the robot passes within half a
  metre of it.
- **Mapping (slam_toolbox).** Scan matching builds the map while driving. Loop
  closing is off because on a self-similar route it could fold identical
  corridors onto each other.

## Nodes

| Node | Package | Description |
|---|---|---|
| `map_server` | `nav2_map_server` | Serves the saved map |
| `amcl` | `nav2_amcl` | Localization on the map |
| `planner_server` | `nav2_planner` | Plans the global path |
| `controller_server` | `nav2_controller` | Follows the path |
| `behavior_server` | `nav2_behaviors` | Recovery motions |
| `bt_navigator` | `nav2_bt_navigator` | Runs the navigation logic trees |
| `velocity_smoother` | `nav2_velocity_smoother` | Smooths the velocity commands |
| `lifecycle_manager_localization` | `nav2_lifecycle_manager` | Starts the localization nodes |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Starts the navigation nodes |
| `slam_toolbox` | `slam_toolbox` | Builds the map (mapping launch only) |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/livox/scan` | `sensor_msgs/LaserScan` | subscribe | Scan for localization, costmaps and mapping |
| `/odom` | `nav_msgs/Odometry` | subscribe | Flat odometry of the robot base |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | subscribe | Localization reset |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | publish | Estimated robot pose |
| `/map` | `nav_msgs/OccupancyGrid` | publish | The saved or live map |
| `/command/robot` | `geometry_msgs/Twist` | publish | Smoothed velocity command to the control node |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action server | Single-goal navigation |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | action server | Waypoint navigation |

## Parameters

### `config/navigation.yaml` — amcl

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `set_initial_pose` | `true` | — | Assume the robot starts at the map origin | Off waits for a manual pose estimate |
| `initial_pose` x, y, z, yaw | `0.0` | m, rad | The assumed start pose | Set if the robot starts elsewhere |
| `alpha1` | `0.02` | — | Expected rotation noise from rotating | Raising any alpha spreads the pose guesses wider. Kept very low so the guesses stay near the accurate odometry and cannot drift onto a look-alike corridor |
| `alpha2` | `0.02` | — | Expected rotation noise from driving | Same as above |
| `alpha3` | `0.02` | — | Expected distance noise from driving | Same as above |
| `alpha4` | `0.02` | — | Expected distance noise from rotating | Same as above |
| `alpha5` | `0.02` | — | Extra sideways noise term | Same as above |
| `laser_model_type` | `likelihood_field` | — | How scan beams are scored against the map | — |
| `laser_likelihood_max_dist` | `2.0` | m | Blur of the map used for scoring | Raising it forgives misalignment more, which resists jumping to a perfectly aligned wrong corridor |
| `laser_max_range` | `30.0` | m | Longest beam used | Keep at the scan range |
| `laser_min_range` | `-1.0` | m | Shortest beam used, -1 takes the scan value | — |
| `max_beams` | `60` | beams | Beams used per update | Raising it localizes better at more computing |
| `min_particles` | `500` | — | Smallest pose guess cloud | Raising it is more robust at more computing |
| `max_particles` | `2000` | — | Largest pose guess cloud | Same trade-off |
| `update_min_d` | `0.15` | m | Distance driven before an update runs | Raising it corrects less often and follows odometry more between updates |
| `update_min_a` | `0.15` | rad | Rotation before an update runs | Same as above |
| `resample_interval` | `1` | updates | Updates between resampling steps | Raising it keeps guess diversity longer |
| `transform_tolerance` | `0.5` | s | How long the published transform stays valid | Raise if transform timeout warnings appear |
| `recovery_alpha_slow` | `0.0` | — | Random restart guesses, slow average | Keep 0 here. Random guesses could land on a look-alike corridor and teleport the pose |
| `recovery_alpha_fast` | `0.0` | — | Random restart guesses, fast average | Keep 0, same reason |
| `pf_err` | `0.05` | — | Allowed particle filter error | — |
| `pf_z` | `0.99` | — | Confidence of the adaptive filter | — |
| `z_hit` | `0.45` | — | Weight of beams that match the map | Raising it trusts the scan more. Slightly lowered so a look-alike corridor cannot outvote the odometry |
| `z_max` | `0.05` | — | Weight of maximum range beams | — |
| `z_rand` | `0.5` | — | Weight of random noise beams | Raising it dilutes the scan influence, which guards against wrong-corridor matches |
| `sigma_hit` | `0.2` | m | Measurement noise of matching beams | — |
| `tf_broadcast` | `true` | — | Publish the map to odom transform | Navigation needs this on |

### `config/navigation.yaml` — planner_server

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `expected_planner_frequency` | `5.0` | Hz | Warn when planning is slower than this | — |
| `tolerance` | `0.25` | m | How close the plan end must be to the goal | Raising it accepts plans that stop short |
| `use_astar` | `false` | — | A* instead of Dijkstra search | Mostly a taste choice |
| `allow_unknown` | `true` | — | Plan through unexplored map cells | Off refuses paths through unknown space |

### `config/navigation.yaml` — controller_server

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `controller_frequency` | `20.0` | Hz | Rate of the control loop | Raising it reacts faster at more computing |
| `min_x_velocity_threshold` | `0.001` | m/s | Commands below this count as zero | — |
| `min_theta_velocity_threshold` | `0.001` | rad/s | Same for rotation | — |
| `failure_tolerance` | `0.3` | s | How long the controller may fail before giving up | Raising it tolerates short failures |
| `required_movement_radius` | `0.3` | m | Movement that counts as progress | Lowering it accepts smaller movements as progress |
| `movement_time_allowance` | `8.0` | s | Time allowed to make that movement before recovery starts | Raising it lets slow careful driving finish. Lowering it detects a stuck robot sooner |
| `xy_goal_tolerance` | `0.15` | m | Position error that counts as arrived | Raising it arrives easier but less exactly |
| `yaw_goal_tolerance` | `0.2` | rad | Heading error that counts as arrived | Same trade-off |
| `stateful` | `true` | — | Do not re-check position while rotating to the final heading | — |

### `config/navigation.yaml` — controller_server FollowPath (Regulated Pure Pursuit)

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `desired_linear_vel` | `0.3` | m/s | Cruise speed | Raising it drives faster everywhere |
| `lookahead_dist` | `0.4` | m | Chase point distance when not speed scaled | Raising it cuts corners more but is smoother |
| `min_lookahead_dist` | `0.25` | m | Smallest chase point distance | Lowering it tracks tighter in corners |
| `max_lookahead_dist` | `0.6` | m | Largest chase point distance | Raising it is smoother in open space |
| `lookahead_time` | `1.5` | s | Chase point distance is speed times this | Raising it looks further ahead at speed |
| `use_velocity_scaled_lookahead_dist` | `true` | — | Scale the chase point with speed | On gives tight slow corners and smooth fast lines |
| `transform_tolerance` | `0.2` | s | Transform age accepted | — |
| `min_approach_linear_velocity` | `0.05` | m/s | Slowest speed while arriving | Raising it does not crawl as much at the goal |
| `approach_velocity_scaling_dist` | `0.6` | m | Distance from the goal where slowing starts | Raising it brakes earlier |
| `use_collision_detection` | `false` | — | Stop for predicted collisions ahead | Kept off. The guaranteed narrowest gaps keep the body inside the forbidden band, so this check would stop the robot in gaps it can actually pass. Avoidance is handled by replanning instead |
| `max_allowed_time_to_collision_up_to_carrot` | `2.0` | s | How far ahead the collision check would look | Only used if the check is on |
| `use_regulated_linear_velocity_scaling` | `true` | — | Slow down in curves | Off keeps full speed in curves |
| `use_cost_regulated_linear_velocity_scaling` | `false` | — | Slow down near obstacles by costmap cost | Off because the local costmap has no cost slope |
| `regulated_linear_scaling_min_radius` | `0.45` | m | Curves tighter than this trigger slowing | Raising it slows for gentler curves too |
| `regulated_linear_scaling_min_speed` | `0.1` | m/s | Slowest speed the curve slowdown may reach | Lowering it creeps slower in tight curves |
| `use_rotate_to_heading` | `true` | — | Stop and rotate on the spot at sharp path turns | This is what makes the corridor-end turns reliable |
| `rotate_to_heading_min_angle` | `0.785` | rad | Path direction jump that triggers rotating on the spot | Lowering it rotates on the spot for gentler turns too |
| `rotate_to_heading_angular_vel` | `0.8` | rad/s | Speed of the on-the-spot rotation | Lowering it is gentler on the odometry during turns |
| `max_angular_accel` | `1.0` | rad/s² | How fast the turn rate may change | Lowering it eases into turns, which keeps the laser odometry stable |
| `allow_reversing` | `false` | — | Allow driving backward along the path | Off because the scanner does not see behind the robot |
| `max_robot_pose_search_dist` | `10.0` | m | Search window when matching the robot onto the path | — |

### `config/navigation.yaml` — local_costmap

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `update_frequency` | `5.0` | Hz | Rebuild rate of the costmap | Raising it reacts faster to new obstacles |
| `publish_frequency` | `2.0` | Hz | Publish rate for visualization | — |
| `rolling_window` | `true` | — | The map moves with the robot | — |
| `width` | `5` | m | Size of the moving window | Raising it sees further at more computing |
| `height` | `5` | m | Same | Same |
| `resolution` | `0.025` | m | Cell size | Small enough that the narrowest gap keeps a drivable centre strip about four cells wide. Raising it saves computing but can swallow narrow gaps |
| `footprint` | 0.50 × 0.50 square | m | Robot outline used for collision | Body plus one centimetre margin per side |
| `max_obstacle_height` | `2.0` | m | Tallest obstacle considered | — |
| `clearing` | `true` | — | The scan can erase obstacles it sees past | Off would leave removed obstacles stuck |
| `marking` | `true` | — | The scan adds obstacles | — |
| `inf_is_valid` | `true` | — | Treat empty scan bins as maximum range so they clear the map | Off would never clear removed obstacles |
| `raytrace_max_range` | `5.5` | m | How far clearing reaches | Keep just above the scanner range |
| `raytrace_min_range` | `0.0` | m | Clearing dead zone | — |
| `obstacle_max_range` | `4.5` | m | How far obstacles are added | Keep just below the scanner range |
| `obstacle_min_range` | `0.0` | m | Marking dead zone | — |
| `cost_scaling_factor` | `3.0` | — | How fast cost decays from walls | Has no effect here because the inflation ends at the body radius |
| `inflation_radius` | `0.25` | m | Forbidden band around walls | Equal to the robot half-width, so beyond the physically impossible zone every cell is free and the narrowest gaps stay drivable. Raising it would flood the gaps with cost |
| `always_send_full_costmap` | `true` | — | Publish the whole map each time | — |

### `config/navigation.yaml` — global_costmap

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `update_frequency` | `1.0` | Hz | Rebuild rate | Enough for replanning at 2 Hz on a small map |
| `publish_frequency` | `1.0` | Hz | Publish rate for visualization | — |
| `resolution` | `0.025` | m | Cell size | Same reasoning as the local costmap |
| `track_unknown_space` | `true` | — | Keep unknown cells distinct from free | — |
| `footprint` | 0.50 × 0.50 square | m | Robot outline | Same as local |
| `map_subscribe_transient_local` | `true` | — | Static layer receives the map reliably | — |
| obstacle layer entries | same as local | — | Live scan marking and clearing | Same values as the local costmap |
| `cost_scaling_factor` | `2.5` | — | How fast cost decays from walls | Lowering it spreads the slope wider so the centre pull reaches across the corridor |
| `inflation_radius` | `0.55` | m | Cost slope width around walls | Wider than the corridor half-width so the cheapest line is the corridor centre and the planned path stays off the walls. The narrowest gaps still keep a drivable channel below lethal cost |
| `always_send_full_costmap` | `true` | — | Publish the whole map each time | — |

### `config/navigation.yaml` — behavior_server

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `cycle_frequency` | `10.0` | Hz | Behavior update rate | — |
| `behavior_plugins` | spin, backup, drive_on_heading, wait | — | Available recovery motions | — |
| `transform_tolerance` | `0.2` | s | Transform age accepted | — |
| `simulate_ahead_time` | `2.0` | s | How far a recovery motion is checked for collision | Raising it is more cautious |
| `max_rotational_vel` | `1.0` | rad/s | Fastest recovery rotation | Matches the controller cap |
| `min_rotational_vel` | `0.2` | rad/s | Slowest recovery rotation | — |
| `rotational_acc_lim` | `1.0` | rad/s² | Rotation ramp | Matched to the rest of the stack so turns stay gentle |

### `config/navigation.yaml` — bt_navigator

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `navigators` | navigate_to_pose, navigate_through_poses | — | The two goal interfaces | Single goal and waypoint list |
| `bt_loop_duration` | `10` | ms | Tick period of the logic tree | — |
| `default_server_timeout` | `20` | s | Wait for action servers at startup | — |

The launch file also passes the two tree files. `behavior_tree.xml` runs a
single goal with replanning at 2 Hz and fast recoveries (clear costmaps, back
up 0.15 m, wait 1 s, up to 10 rounds). `navigate_through_poses_bt.xml` does
the same for a waypoint list and drops every waypoint once the robot passes
within 0.5 m of it.

### `config/navigation.yaml` — velocity_smoother

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `smoothing_frequency` | `20.0` | Hz | Output rate | Raising it gives finer ramps |
| `scale_velocities` | `false` | — | Scale both axes together when limiting | — |
| `feedback` | `OPEN_LOOP` | — | Ramp from the last command instead of measured speed | — |
| `max_velocity` | `[0.3, 0.0, 1.0]` | m/s, m/s, rad/s | Forward, sideways and turn speed caps | The system-wide speed limits |
| `min_velocity` | `[-0.3, 0.0, -1.0]` | m/s, m/s, rad/s | Reverse caps | — |
| `max_accel` | `[0.5, 0.0, 1.0]` | m/s², m/s², rad/s² | Acceleration limits | Lowering the last value eases into turns, which keeps the laser odometry stable |
| `max_decel` | `[-0.7, 0.0, -1.0]` | same | Braking limits | Raising the first value stops harder |
| `odom_duration` | `0.1` | s | Odometry averaging window | — |
| `deadband_velocity` | `[0.0, 0.0, 0.0]` | same as velocity | Commands below this are zeroed | — |
| `velocity_timeout` | `1.0` | s | Stop when no command arrives within this time | Lowering it stops sooner when upstream dies |

### `config/map.yaml` (slam_toolbox, used while mapping)

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `solver_plugin` | CeresSolver | — | Optimizer of the pose graph | — |
| `ceres_linear_solver` | SPARSE_NORMAL_CHOLESKY | — | Solver internals | — |
| `ceres_preconditioner` | SCHUR_JACOBI | — | Solver internals | — |
| `ceres_trust_strategy` | LEVENBERG_MARQUARDT | — | Solver internals | — |
| `mode` | `mapping` | — | Build a map rather than only localize | — |
| `debug_logging` | `false` | — | Verbose output | — |
| `throttle_scans` | `1` | — | Process every Nth scan | Raising it lightens computing but maps coarser in time |
| `transform_publish_period` | `0.02` | s | Map to odom transform publish period | — |
| `map_update_interval` | `2.0` | s | How often the drawn map is rebuilt | Lowering it refreshes the picture faster at more computing |
| `resolution` | `0.05` | m | Map cell size | Lowering it maps finer at more memory |
| `max_laser_range` | `30.0` | m | Scan range used for mapping | Keep equal to the converter range |
| `minimum_time_between_scans` | `0.1` | s | Scans arriving faster are dropped | — |
| `transform_timeout` | `0.2` | s | Transform lookup wait | — |
| `tf_buffer_duration` | `30.0` | s | Transform history kept | — |
| `stack_size_to_use` | `40000000` | bytes | Thread stack for saving big maps | — |
| `enable_interactive_mode` | `true` | — | Allow moving graph nodes from RViz | — |
| `minimum_travel_distance` | `0.15` | m | Distance before a new scan joins the map | Raising it maps sparser |
| `minimum_travel_heading` | `0.15` | rad | Rotation before a new scan joins | Same |
| `scan_buffer_size` | `15` | scans | Recent scans kept for matching | — |
| `scan_buffer_maximum_scan_distance` | `30.0` | m | Keep at or above the laser range | — |
| `link_match_minimum_response_fine` | `0.12` | — | Match quality needed to link scans | Raising it links only confident matches |
| `link_scan_maximum_distance` | `1.5` | m | Furthest scans considered for linking | — |
| `do_loop_closing` | `false` | — | Recognize revisited places | Kept off. On a route of identical-looking legs it would match one leg to another and fold the map |
| `loop_search_maximum_distance` | `1.5` | m | Search radius for revisits | Small so only true revisits could match |
| `loop_match_minimum_chain_size` | `15` | scans | Consecutive scans needed for a revisit | Raising it demands more evidence |
| `loop_match_maximum_variance_coarse` | `3.0` | — | Reject uncertain matches | — |
| `loop_match_minimum_response_coarse` | `0.35` | — | Match quality needed, coarse | — |
| `loop_match_minimum_response_fine` | `0.45` | — | Match quality needed, fine | — |
| `correlation_search_space_dimension` | `0.5` | m | Local match search window | — |
| `correlation_search_space_resolution` | `0.01` | m | Search grid step | — |
| `correlation_search_space_smear_deviation` | `0.1` | m | Search smoothing | — |
| `loop_search_space_dimension` | `8.0` | m | Revisit search window | — |
| `loop_search_space_resolution` | `0.05` | m | Revisit grid step | — |
| `loop_search_space_smear_deviation` | `0.03` | m | Revisit smoothing | — |
| `distance_variance_penalty` | `0.5` | — | Cost of deviating from odometry in position | Raising it trusts odometry more |
| `angle_variance_penalty` | `1.0` | — | Cost of deviating in angle | Raising it trusts odometry more |
| `fine_search_angle_offset` | `0.00349` | rad | Fine angular search span | — |
| `coarse_search_angle_offset` | `0.349` | rad | Coarse angular search span | — |
| `coarse_angle_resolution` | `0.0349` | rad | Coarse angular step | — |
| `minimum_angle_penalty` | `0.9` | — | Floor of the angle penalty | — |
| `minimum_distance_penalty` | `0.5` | — | Floor of the distance penalty | — |
| `use_response_expansion` | `true` | — | Widen the search when matching poorly | — |

### `map/map.yaml` (the saved map)

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `image` | `map.pgm` | — | The map picture file | — |
| `mode` | `trinary` | — | Cells are free, occupied or unknown | — |
| `resolution` | `0.05` | m | Size of one map pixel | Set by the mapping resolution |
| `origin` | `[-2.51, -4.42, 0]` | m, m, rad | World position of the lower-left pixel | Written by the map saver |
| `negate` | `0` | — | Invert black and white | — |
| `occupied_thresh` | `0.65` | — | Darkness above which a pixel is a wall | — |
| `free_thresh` | `0.25` | — | Brightness below which a pixel is free | — |
