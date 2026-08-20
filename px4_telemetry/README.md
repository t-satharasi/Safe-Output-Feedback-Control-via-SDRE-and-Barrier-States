# PX4_Telemetry
ROS2 node that handles conversions from global gpos fixes in LLA to the local Autonomy Park frame. It converts a GPS fix from WGS-84 into UTM easting/northing, 
subtracts out the park x/y origin, then rotates about the heading offset origin_r.

Orientation is taken from `global_position/local` (odometry), and global position it taken from `global_position/global`.

## Dependencies
 
- ROS2 (tested on Humble)
- `mavros`, `mavros_msgs`
- `geodesy`, `geographiclib` (egm96-5 geoid dataset required at runtime)
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `geographic_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
 
## Build
 
```bash
colcon build --packages-select px4_telemetry
source install/setup.bash
```

### `park_coordinates.yaml`
 
Defines the park origin in UTM and its heading offset relative to UTM north.
 
| Parameter | Description |
|---|---|
| `origin_x` | Park origin UTM easting (meters) |
| `origin_y` | Park origin UTM northing (meters) |
| `origin_z` | Park origin altitude AMSL (meters) |
| `origin_r` | Park heading offset from UTM north (radians) |
| `utm_zone` | UTM zone number |
| `utm_band` | UTM band letter |

### Runtime parameter
 
| Parameter | Default | Description |
|---|---|---|
| `sim_mode` | `false` | Use monotonic altitude (sim) instead of local altitude (hardware) |
