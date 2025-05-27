# Run ALFA extensions

To run ALFA extensions, execute the <b>ros2 run</b> command followed by the package name and extension name:

```sh
ros2 run <ext_name> <ext_name> /<topic-name>
```

Each Extension provides its own output.

## Node Topics and Services

To check available topics:

```sh
ros2 topic list
```

Will output:

```sh
/<extension_name>_alive
/<extension_name>_metrics
/<extension_name>_pointcloud
```

Check the output of a topic:

```sh
ros2 topic echo /<extension_name>_alive 
```

To check available Extension services:

```sh
ros2 service list
```

Some Extensions provide parameters change (available in the /<extension_name>_alive topic):

```sh
ros2 param set /<extension_name> <parameter> <new_value>
```

## Point Cloud Visualization using ALFA-Monitor

At your directory with a ros2bag dataset:

```sh
ros2 bag play <dataset_in_ros2bag format>.db3 --loop
```

The terminal where the Extension is running should output:

```sh
Point cloud received
```

Open a new terminal and go to your ROS2 workspace. Source the environment:

```sh
source ./install/setup.bash 
```

Launch the alfa-monitor:

```sh
ros2 run alfa-monitor alfa-monitor
```

To get further details on how to use the tool go to the [ALFA-Monitor guide](monitor-guide.md).

## Point Cloud Visualization using RVIZ

Before using the rviz2 tool, provided by ROS2, to visualize the point could output we need to apply a window transform for the output of ALFA extensions and the bag point cloud:

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "<extension_topic_name>" 
```

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "velodyne" 
```

Run RVIZ Tool

```sh
ros2 run rviz2 rviz2
```

Now in the RViz window you can:

- Add a new visualization
- Add by topic
- add /velodyne_points to see the original point cloud
- add /<extension_topic_name> to see the Extension point cloud
