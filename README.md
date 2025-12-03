# Tracter Odometer

A ROS 2 package that implements a kinematic model for a Tractor with N Drawbar Trailers. It calculates and publishes the state (articulation angles) of the entire trailer chain based on the tractor's odometry input.

## Features

*   **N-Trailer Support**: Configurable number of trailers with customizable dimensions.
*   **Odometry Input**: Subscribes to `/localization/kinematic_state` (Autware) to track the tractor's motion.
*   **TF Broadcasting**: Broadcasts TF transforms for every trailer unit (`trailer_i_dolly`, `trailer_i_base_link`).
*   **Visualization**: Publishes `MarkerArray` for rich 3D visualization in RViz (Tractor, Trailers, Wheels, Drawbars).
*   **Configurable**: All parameters (dimensions, colors) are loaded from a YAML configuration file.

## Installation

1.  **Clone the repository** into your workspace `src` folder:
    ```bash
    cd ~/tracter_ws/src
    git clone https://github.com/nontanan-linux/tracter_odometer.git
    ```

2.  **Build the package**:
    ```bash
    cd ~/tracter_ws
    colcon build --packages-select tracter_odometer
    ```

3.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Usage

### Launching (Recommended)

Use the provided launch file to start the node with the default configuration and RViz visualization.

**Python Launch:**
```bash
ros2 launch tracter_odometer tracter_odometer.launch.py
```

**XML Launch:**
```bash
ros2 launch tracter_odometer tracter_odometer.launch.xml
```

### Running Node Manually

If you prefer to run the node directly (make sure to load parameters manually or via a params file):
```bash
ros2 run tracter_odometer odometer_node --ros-args --params-file $(ros2 pkg prefix tracter_odometer)/share/tracter_odometer/config/tracter_config.yaml
```

## Configuration

The package is configured via `config/tracter_config.yaml`.

### Vehicle Parameters
*   `L0`: Tractor wheelbase [m].
*   `dt`: Simulation time step [s].
*   `trailer_L_bars`: List of drawbar lengths for each trailer [m].
*   `trailer_L_trls`: List of trailer body lengths (hitch to axle) [m].

### Hitch Calculation
The hitch offset (distance from axle to hitch point) is calculated as `Overhang + Tail Length`.
*   `tractor_tail_length`: Distance from Tractor Rear Face to Hitch 1 [m].
*   `trailer_tail_length`: Distance from Trailer Rear Face to Hitch [m].

### Visualization Dimensions
You can customize the size of the vehicle elements in `config/tracter_config.yaml`:
*   `tractor_width`: Width of the tractor [m].
*   `tractor_front_overhang`: Distance from Front Axle to Front Face [m].
*   `tractor_overhang`: Distance from Rear Axle to Rear Face (Rear Overhang) [m].
*   **Note**: Tractor Total Length is calculated as `L0 + tractor_overhang + tractor_front_overhang`.

*   `trailer_width`, `trailer_body_len`, `trailer_overhang`
*   `wheel_diam`, `wheel_width`, `track_width`

### Visualization Colors
Colors are defined as RGBA vectors `[r, g, b, a]`.
*   `color_tractor`: Color of the tractor body.
*   `color_trailer`: Color of the trailer bodies.
*   `color_wheel`: Color of the wheels.
*   `color_drawbar`: Color of the drawbars and hitch points.

## Nodes

### `odometer_node`

#### Subscribed Topics
*   `/localization/kinematic_state` ([nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))
    *   Uses `pose.pose` for Tractor position ($x, y$) and orientation ($\theta$).
    *   Uses `twist.twist` to calculate steering angle ($\delta$).

#### Published Topics
*   `/joint_states` ([sensor_msgs/msg/JointState](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg))
    *   Articulation angles: `steering_joint`, `hitch_i_joint`, `dolly_i_joint`.
*   `/visualization_marker_array` ([visualization_msgs/msg/MarkerArray](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/MarkerArray.msg))
    *   3D markers for RViz visualization.
*   `/tf` ([tf2_msgs/msg/TFMessage](https://github.com/ros2/common_interfaces/blob/master/tf2_msgs/msg/TFMessage.msg))
    *   Transforms for all trailer links relative to `map`.

## Visualization

### RViz Setup
The launch files automatically open RViz with a pre-configured view (`rviz/tracter.rviz`).

If setting up manually:
1.  **Fixed Frame**: `map`
2.  **Add Display**: `TF` to see coordinate frames.
3.  **Add Display**: `MarkerArray` subscribed to `/visualization_marker_array`.

### Visual Elements
*   **Tractor**: Orange box (default).
*   **Trailers**: Blue boxes (default).
*   **Wheels**: White boxes (default).
*   **Drawbars**: Light Blue lines (default).

## Kinematics

The system uses a recursive bicycle model.
*   **Tractor**: Standard bicycle model.
*   **Trailers**: Each trailer is modeled as a drawbar connected to a trailer axle. The position of the $i$-th trailer is derived from the $(i-1)$-th unit's position and the hitch constraints.

For more details on the math, refer to the `kinematic_model.py` source code.
