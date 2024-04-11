# Mocap to PX4 ROS2 Package

## Overview
The Mocap to PX4 ROS2 package is designed to bridge motion capture (MOCAP) data with the PX4 flight controller. It subscribes to MOCAP rigid body data, transforms the information, and publishes it as odometry data compatible with PX4.

## Prerequisites
- ROS2 (Robot Operating System 2)
- PX4 firmware configured to receive odometry data

## Installation
1. Clone this repository to your ROS2 workspace:

    ```bash
    git clone https://github.com/hpaul360/mocap_to_px4.git
    ```

2. Build the package:

    ```bash
    colcon build --symlink-install
    ```

3. Source the ROS2 setup file:

    ```bash
    source install/setup.bash
    ```

## Usage
1. Ensure your PX4 flight controller is configured to receive odometry data.
2. Launch the `mocap_to_px4` node:

    ```bash
    # if body_id argument is not passed, the default body_id=1 is used
    ros2 launch mocap_to_px4 converter_launch.py body_id:=5
    ```

3. The node will subscribe to MOCAP data, transform it, and publish odometry data to PX4.

## Parameters
- Set the ID of the rigid body (body_id) to subscribe from MOCAP, as argument when launching the node. Adjust as needed.

## Transforms
- Adjusts the sign of the MOCAP position along the Y-up frame to match PX4's coordinate frame.
- Adjusts the quaternion orientation to match PX4's expectations.

## `VehicleOdometry` Message Fields
- `timestamp`, `timestamp_sample`, `pose_frame`, `position`, `q`: Representing the odometry data.
- `velocity_frame`, `velocity`, `angular_velocity`, `position_variance`, `orientation_variance`, `velocity_variance`: Additional information.

## License
This software is released under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author
- [Hannibal Paul](https://github.com/hpaul360)

## Issues and Contributions
- Report any issues or contribute to the development of this package on [GitHub](https://github.com/hpaul360/mocap_to_px4).

## Contact
- For questions or further assistance, feel free to contact the author [Hannibal Paul](https://hannibalpaul.com/).
