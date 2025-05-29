# ALFA Extensions

This directory provides ready-to-use ALFA extensions. They can be used as a starting point for development or for testing the different ALFA setups.

---

## Available Extensions (Summary)

| Extension            | Description                                                                                      |
|----------------------|--------------------------------------------------------------------------------------------------|
| [Dummy](#dummy-extension)                  | Forwards input pointcloud as-is to output topic. Useful as a template for new extensions. |
| [Distance Filter](#distance-filter-extension)       | Filters points between a configurable min and max range. Defaults: 5m–20m.                 |
| [DIOR](#dior-extension)                    | Removes dynamic low-intensity outliers using the DIOR algorithm.                          |
| [Octree Compression](#octree-compression-encoder-and-decoder-extensions) | Compresses and decompresses point clouds using custom octree-based encoding.              |
| [PCL Octree Compression](#pcl-octree-compression-encoder-and-decoder-extensions) | Compresses and decompresses point clouds using PCL’s octree-based codec.                  |
| [FOG-zip](#fog-zip-compression-encoder-and-decoder-extensions) | Compresses and decompresses point clouds FOG-zip, a octree-based compression algorithm based on Huffman coding.|
| [Anand](#anand-extension)                  | Segments ground and non-ground points. Compatible with `alib_metrics` for evaluation.     |
| [Patchwork](#patchwork-extension) | Segments ground and non-ground points. (TODO)    |
| [Patchworkpp](#patchworkpp-extension) | Segments ground and non-ground points. (TODO)     |

---

## Tested Datasets

ALFA was tested with the following datasets:

- [Lincoln MKZ Dataset by Richard Kelley](https://richardkelley.io/data) – Sensor: Velodyne VLP-16 Puck
- [UMA-SAR](https://www.uma.es/robotics-and-mechatronics/cms/menu/robotica-y-mecatronica/datasets/) – Sensor: Velodyne HDL-32E
- [Semantic KITTI](http://www.semantic-kitti.org/dataset.html#download) – Sensor: Velodyne HDL-64E
- [Zenseact](https://www.zenseact.com/) – Sensor: Velodyne VLS-128

**Note:** The datasets are not in the ROS2 bag format. You need to convert them manually. You can refer to:
- <https://github.com/tomas789/kitti2bag> or
- <https://github.com/amslabtech/semantickitti2bag> (for Semantic KITTI) – these convert only to ROS1. A second step is needed to convert to ROS2.

If you need ready-to-use ROS2 bags, feel free to contact us.

For quick testing with Sequence 0 of SemanticKITTI, a small `ros2bag` is available inside the [ALFA-Monitor](https://github.com/alfa-project/alfa-monitor) repository, or you can download the full bag from here:  
[Google Drive Link](https://drive.google.com/file/d/1OZQD10dJnyoZhAmgj--DWqrwAuQeNiHU/view?usp=sharing)

---

## Usage and Development

More information about how to run and develop your own ALFA extensions can be found in the [ALFA Extensions User Guide](../docs/guides/extensions_user_guide.md).

---

## Extensions Details

### Dummy Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_dummy) | [Hardware](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_dummy)
- **Purpose**: Demonstrates how ALFA extensions are structured and used.
- **Default Input**: Subscribes to `/velodyne_points` (`sensor_msgs/PointCloud2`)
- **Output**: Publishes to `/ext_dummy_pointcloud` (same point cloud, unmodified)

---

### Distance Filter Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_distance_filter) | [Hardware](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_distance_filter)
- **Purpose**: Filters points based on their distance from the sensor.
- **Default Input**: Subscribes to `/velodyne_points` (`sensor_msgs/PointCloud2`)
- **Parameters**:
  - `min_distance` (default: 5.0 meters)
  - `max_distance` (default: 20.0 meters)
- **Output**: Publishes filtered point cloud to `/ext_distance_filter_pointcloud`

---

### DIOR Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_dior) | [Hardware](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_dior)
- **Reference**: [DIOR Algorithm](https://ieeexplore.ieee.org/document/9643022)
- **Purpose**: Implements the DIOR denoising algorithm to remove low-intensity noise under adverse conditions.
- **Default Input**: Subscribes to `/velodyne_points` (`sensor_msgs/PointCloud2`)
- **Parameters**: TODO
- **Output**: Publishes denoised point cloud to `/ext_dior_pointcloud`

---

### Octree Compression Encoder and Decoder Extensions

- **Source files**:
  - **Encoder**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_octree_compression_encoder) 
  - **Decoder**: [Software](https://github.com/alfa-project/alfa-framework/tree/main/extensions/sw/ext_octree_compression_decoder)
- **Purpose**: Compresses and decompresses LiDAR point cloud data using a custom octree encoding.
- **Encoder**:
  - **Default Input**: Subscribes to `/velodyne_points`
  - **Parameters**: TODO
  - **Output**: Publishes compressed bitstream to `/ext_octree_compression_encoder_pointcloud`
- **Decoder**:
  - **Default Input**: Subscribes to `/ext_octree_compression_encoder_pointcloud`
  - **Parameters**: TODO
  - **Output**: Publishes reconstructed point cloud to `/ext_octree_compression_decoder_pointcloud`

---

### PCL Octree Compression Encoder and Decoder Extensions

- **Source files**:
  - **Encoder**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_pcl_octree_compression_encoder)
  - **Decoder**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_pcl_octree_compression_decoder)
- **Reference**: TODO
- **Purpose**: Uses the [PCL native octree compression codec](https://pointclouds.org/documentation/tutorials/compression.html) to compress and decompress LiDAR data.
- **Encoder**:
  - **Default Input**: Subscribes to `/velodyne_points`
  - **Parameters**: TODO
  - **Output**: Publishes compressed bitstream to `/ext_pcl_octree_compression_encoder_pointcloud`
- **Decoder**:
  - **Default Input**: Subscribes to `/ext_pcl_octree_compression_encoder_pointcloud`
  - **Parameters**: TODO
  - **Output**: Publishes reconstructed point cloud to `/ext_pcl_octree_compression_decoder_pointcloud`

---

### FOG-zip Compression Encoder and Decoder Extensions

- **Source files**:
  - **Encoder**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_fogzip_encoder) | [Hardware](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_fog_encoder)
  - **Decoder**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_fogzip_decoder)
- **Reference**: TODO
- **Purpose**: Compresses and decompresses LiDAR point cloud data using a fast octree-based encoding based on a simplified Huffman coding.
- **Encoder**:
  - **Default Input**: Subscribes to `/velodyne_points`
  - **Parameters**: TODO
  - **Output**: Publishes compressed bitstream to `/ext_fogzip_encoder_pointcloud`
- **Decoder**:
  - **Default Input**: Subscribes to `/ext_fogzip_encoder_pointcloud`
  - **Parameters**: TODO
  - **Output**: Publishes reconstructed point cloud to `/ext_fogzip_decoder_pointcloud`

---

### Anand Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_anand)
- **Reference**: [Anand et al. Ground Segmentation](https://ieeexplore.ieee.org/abstract/document/9558794)
- **Purpose**: Segments the point cloud into ground and non-ground points.
- **Default Input**: Subscribes to `/velodyne_points`
- **Parameters**: TODO
- **Output**: Publishes segmented point cloud to `/ext_anand_pointcloud`
  - Can be configured to show only ground or only non-ground points
- **Integration**: Supports evaluation using [alib_metrics](https://github.com/alfa-project/alfa-framework/tree/main/libs/sw/alib_metrics)
- **Dataset Support**: Best evaluated with the SemanticKITTI dataset

---

### Patchwork Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_patchwork)
- **Reference**: TODO
- **Purpose**: Segments the point cloud into ground and non-ground points.
- **Default Input**: Subscribes to `/velodyne_points`
- **Parameters**: TODO
- **Output**: Publishes segmented point cloud to `/ext_patchwork_pointcloud`
  - Can be configured to show only ground or only non-ground points
- **Dataset Support**: Best evaluated with the SemanticKITTI dataset

---

### Patchworkpp Extension

- **Source files**: [Software](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_patchworkpp)
- **Reference**: TODO
- **Purpose**: Segments the point cloud into ground and non-ground points.
- **Default Input**: Subscribes to `/velodyne_points`
- **Parameters**: TODO
- **Output**: Publishes segmented point cloud to `/ext_patchworkpp_pointcloud`
  - Can be configured to show only ground or only non-ground points
- **Dataset Support**: Best evaluated with the SemanticKITTI dataset