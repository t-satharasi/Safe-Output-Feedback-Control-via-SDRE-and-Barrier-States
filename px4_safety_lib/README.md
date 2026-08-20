# px4_safety_lib

Safety library employing potential field based safety for velocity-based commands. 

Packages using this library will instanciate a PX4_Safety object and call the compute_safe_cmd_vel(), passing in an unfiltered command velocity. 
This function will take the agent's current pose and the unfiltered Twist message, compute influene vectors from each park boundary and added obstacles, 
then output the corrected velocity. 

Influence parameters can be configured from `param/safety_config.yaml`.

## Parameters
 
### `safety_config.yaml`
 
| Parameter | Default | Description |
|---|---|---|
| `safety.min_x/max_x` | -30.0 / 31.0 m | Fence bounds in park-frame X |
| `safety.min_y/max_y` | -7.8 / 5.0 m | Fence bounds in park-frame Y |
| `safety.min_z/max_z` | 1.0 / 4.0 m | Fence bounds in park-frame Z |
| `safety.max_influence` | 1.0 | Maximum repulsive velocity correction (m/s) |
| `safety.fence_a/b/p` | 0.005 / 2.0 / 1.0 | Fence influence function gain, exponent, activation distance |
| `safety.obs_a/b/p` | 0.5 / 2.0 / 1.0 | Obstacle influence function gain, exponent, activation distance |
| `safety.enable_viz` | `true` | Publish obstacle positions as a `MarkerArray` for RViz2 |

### `obstacles.yaml`
 
```yaml
/**:
  ros_parameters:
    agent_ids: ['agent_1', 'agent_2', 'agent_3']
```
 
Lists the other agents to treat as dynamic obstacles. Each ID must have an active `px4_telemetry` node publishing to `/<id>/autonomy_park/pose`.
