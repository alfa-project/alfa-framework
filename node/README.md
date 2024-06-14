# **ALFA-Node**

<p align="justify"> <b> ALFA Node</b>  is a ROS-based node that supports the ALFA Extensions to facilitate the development of point cloud processing algorithms. The ALFA Node receives configurations through the ROS2 set parameters command (<b>ros2 param set &ltnode_name&gt &ltparameter_name&gt &ltvalue&gt</b>), outputs real-time metrics (<b>/NODE_NAME_metrics</b>) and publishes the processed point cloud data into a new <b>PointCloud2</b> ROS2 topic (<b>NODE_NAME_pointcloud</b>). The point cloud data can be retrieved from a real LiDAR sensor connected to the platform or from a public dataset thtough a ROS2 Bag (rosbag tool). Additionally, the Desktop version of the framework supports the <b>ALFA Monitor</b>, a GUI tool specially designed to configure the ALFA Extension, visualize point cloud data, and read real-time metrics from every ALFA Node running both in the Desktop and the Embedded System.</p></a>

#### Changing this package is not recomended!

