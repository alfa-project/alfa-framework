# **Distance Filter Extension - Software**

This extension is used to demonstrate how ALFA Extensions are built and used. It simply subscribes a Pointcloud2 topic called <b>/velodyne_points</b>  topic and outputs the same pointcloud into <b>/dummy_pointcloud</b> topic.

Run the Extension:
```sh
ros2 run ext_distance_filter  ext_distance_filter /velodyne_points
```
Output:
```sh
Starting ALFA node with the following settings:
Subscriber topic: /velodyne_points
Name of the node: ext_distance_filter
Extension ID: 0
Pointcloud ID: 0
Hardware Driver (SIU): 0
Hardware Extension: 0
--------------------------------------------------------
```

Check Extension topics:

```sh
ros2 topic list
```

Output:
```sh
/ext_distance_filter_alive
/ext_distance_filter_metrics
/ext_distance_filter_pointcloud
```

Distance Filter Extension metrics:
```sh
ros2 topic echo /ext_distance_filter_metrics 
```
Output:
```sh
message_tag: ''
metrics:
- metric_name: Full processing time
  metric: 7453.0
  units: us
- metric_name: Handler processing time
  metric: 2233.0
  units: us
- metric_name: Processed Points
  metric: 120209.0
  units: points
---
```

Distance Filter Extension services:
```sh
ros2 service list
```
Output:
```sh
/ext_distance_filter/describe_parameters
/ext_distance_filter/get_parameter_types
/ext_distance_filter/get_parameters
/ext_distance_filter/list_parameters
/ext_distance_filter/set_parameters
/ext_distance_filter/set_parameters_atomically
```

Distance Filter Extension parameters:
```sh
ros2 topic echo /ext_distance_filter_alive
```
Output:
```sh
---
node_name: ext_distance_filter
node_type: extension
config_service_name: ext_distance_filter_settings
current_status: 0
config_tag: Configuration
default_configurations:
- config_name: min_distance
  config: 5.0
- config_name: max_distance
  config: 20.0
---
```

The ext_distance_filter accepts changing the **min_distance** and **max_distance** parameters. To change the min distance to 1.0, type:
```sh
ros2 param set /ext_distance_filter min_distance 1.0
```
```sh
ros2 param set /ext_distance_filter max_distance 10.0
```

Change these parameters to see the output changing in the ALFA Monitor or RViz tool.

<p align="center">
<img src="../../docs/Images/distance_filter.png" alt="distance-filter" width="840" />
</p>