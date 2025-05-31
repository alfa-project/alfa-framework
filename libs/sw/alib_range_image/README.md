# **Dummy Extension - Software**

This extension is used to demonstrate how ALFA Extensions are built and used. It simply subscribes a Pointcloud2 topic called <b>/velodyne_points</b>  topic and outputs the same pointcloud into <b>/dummy_pointcloud</b> topic.

Run the Extension:
```sh
ros2 run ext_dummy ext_dummy /velodyne_points
```
Output:
```sh
--------------------------------------------------------
Starting ALFA node with the following settings:
Subscriber topic: /velodyne_points
Name of the node: ext_dummy
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
/ext_dummy_alive
/ext_dummy_metrics
/ext_dummy_pointcloud
```

Dummy Extension metrics:
```sh
ros2 topic echo /ext_dummy_metrics 
```

Dummy Extension services and parameters:
```sh
No Services or parameters available
```