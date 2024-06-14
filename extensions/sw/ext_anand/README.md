# **Anand et al. - Software**

This extension implements the ground segmentation algorithm proposed by Anand et al. [1]. By default, it subscribes a Pointcloud2 topic called <b>/velodyne_points</b> topic and outputs a point cloud into <b>/ext_anand_pointcloud</b> topic. For the output of all performance metrics, it is recommended a labeled dataset. This extension was tested with the SemanticKITTI dataset.

##### [1] B. Anand, M. Senapati, V. Barsaiyan and P. Rajalakshmi, "LiDAR-INS/GNSS-Based Real-Time Ground Removal, Segmentation, and Georeferencing Framework for Smart Transportation," in IEEE Transactions on Instrumentation and Measurement, vol. 70, pp. 1-11, 2021, Art no. 8504611, doi: 10.1109/TIM.2021.3117661.

Run the Extension:
```sh
ros2 run ext_anand ext_anand /velodyne_points
```
Output:
```sh
--------------------------------------------------------
Starting ALFA node with the following settings:
Subscriber topic: /velodyne_points
Name of the node: ext_anand
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
/ext_anand_alive
/ext_anand_metrics
/ext_anand_pointcloud
```

Dummy Extension metrics:
```sh 
No Extension metrics are available

Note: To avoid extra processing overheads, this extension outputs its metrics after processing all the point cloud sequence by pressing CTRL+C.
```

Built-in metrics output:
```sh 
------ METRICS -------
Number of frames: 4541
----------------------
TP: 47118 +/- 10936
FP: 4691 +/- 1521
TN: 66294 +/- 9762
FN: 1245 +/- 1137
----------------------
True Positive Rate: 97.41 +/- 2.166 %
True Negative Rate: 93.37 +/- 1.924 %
Positive Predictive Value: 90.44 +/- 3.963 %
Negative Predictive Value: 98.12 +/- 1.856 %
F1-Score: 93.74 +/- 2.411 %
Accuracy: 95.02 +/- 1.42 %
IoUg: 88.31 +/- 4.153 %
----------------------
Total points [In]: 121494 +/- 4100
Total points [Out]: 121494 +/- 4100
Removed points: 51810 +/- 10487
----------------------
Points outside of grid: 0 +/- 0
Removed points (Handler): 52132 +/- 10490
----------------------
Handler Time: 8.993 +/- 2.685 ms
Full Processing Time: 13.7 +/- 2.829 ms
-----------------------
Points outside of grid: 0
Removed points: 52132
----------------------
```

Dummy Extension services and parameters:
```sh
No Services or parameters re available
```