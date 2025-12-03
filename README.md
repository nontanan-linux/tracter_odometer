# Tracter Odometer

A ROS 2 package that implements a kinematic model for a Tractor with N Drawbar Trailers. It calculates and publishes the state (articulation angles) of the entire trailer chain based on the tractor's velocity and steering input.

## Overview

This package provides the `odometer_node`, which:
1.  Subscribes to tractor control inputs (`/cmd_vel`).
2.  Simulates the kinematics of the tractor and $N$ trailers using a recursive bicycle model.
3.  Publishes the joint angles (hitch and dolly angles) for visualization and state estimation.

## Nodes

### `odometer_node`

#### Subscribed Topics
*   `/localization/kinematic_state` ([nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg))
    *   `twist.twist.linear.x`: Tractor longitudinal velocity ($v_0$).
    *   `twist.twist.angular.z`: Used to calculate steering angle ($\delta = \arctan(\frac{\omega L}{v})$).

#### Published Topics
*   `/joint_states` ([sensor_msgs/msg/JointState](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/JointState.msg))
    *   Publishes the articulation angles for the tractor steering and all trailer connections.
    *   Joint names: `steering_joint`, `hitch_i_joint`, `dolly_i_joint`.

#### Parameters
*   `L0` (double, default: 2.0): Tractor wheelbase [m].
*   `dt` (double, default: 0.05): Simulation time step [s].
*   *Trailer parameters are currently hardcoded in the node but can be extended to load from a YAML file.*

## Usage

1.  **Build the package**:
    ```bash
    colcon build --packages-select tracter_odometer
    ```

2.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

3.  **Run the node**:
    ```bash
    ros2 run tracter_odometer odometer_node
    ```

4.  **Send commands**:
    You can publish to `/cmd_vel` to drive the system:
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.5}}"
    ```

5.  **Visualize**:
    Check the output on `/joint_states`:
    ```bash
    ros2 topic echo /joint_states
    ```

6.  **Visualize in RViz**:
    Launch RViz with the provided configuration to see the TF frames:
    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tracter_odometer)/share/tracter_odometer/rviz/tracter.rviz
    ```
    *   **Fixed Frame**: `map`
    *   **TF**: You should see the following frames:
        *   `map` -> `base_link` (from Autoware)
        *   `trailer_i_dolly` (Drawbar end)
        *   `trailer_i_base_link` (Trailer axle)

    **Manual RViz Setup:**
    If you are not using the provided configuration file:
    1.  Run `ros2 run rviz2 rviz2`.
    2.  Set **Fixed Frame** to `map`.
    3.  Click the **Add** button (bottom left).
    4.  Select **TF** from the list and click **OK**.
    5.  Expand the **TF** settings to adjust marker scale and visibility.
    6.  Click **Add** again, select **MarkerArray**, and set the topic to `/visualization_marker_array`.
    
    **Visualization Details:**
    *   **Tractor**: Orange box with wheels (Configurable via `color_tractor`).
    *   **Trailers**: Blue boxes with wheels (Configurable via `color_trailer`).
    *   **Drawbars**: Lines connecting the units (Configurable via `color_drawbar`).
    *   **Wheels**: Grey boxes indicating wheel position and steering (Configurable via `color_wheel`).

    You can change these colors in `config/tracter_config.yaml`. Format: `[r, g, b, a]`.
