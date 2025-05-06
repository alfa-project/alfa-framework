/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <bitset>
#include <cmath>
#include <vector>

#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "alib_octree.hpp"

#define NODE_NAME "ext_pcl_octree_compression"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0

// Conversion function from AlfaPoint to pcl::PointXYZ
pcl::PointXYZ convertToPCLPoint(const AlfaPoint& point) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    return pcl_point;
}

// Conversion from pcl::PointXYZ back to AlfaPoint
AlfaPoint convertToAlfaPoint(const pcl::PointXYZ& pcl_point) {
    AlfaPoint point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    point.custom_field = 0;  // Set default value for custom_field
    return point;
}
// Check if it is inside BB
bool is_point_inside_BB(AlfaPoint point, AlfaBB bounding_box) {
    if (point.x >= bounding_box.min_x && point.x <= bounding_box.max_x &&
        point.y >= bounding_box.min_y && point.y <= bounding_box.max_y &&
        point.z >= bounding_box.min_z && point.z <= bounding_box.max_z)
        return true;
    else
        return false;
}

/**
 * @brief Handler function that copy input pointcloud into output pointcloud
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN);
#else
  float octreeResolution = 0.05f;  // 5 cm octree resolution
  auto octree_depth = node->get_extension_parameter("octree_depth");
  AlfaBB bb(node->get_extension_parameter("min_bounding_box_x"),
            node->get_extension_parameter("min_bounding_box_y"),
            node->get_extension_parameter("min_bounding_box_z"),
            node->get_extension_parameter("max_bounding_box_x"),
            node->get_extension_parameter("max_bounding_box_y"),
            node->get_extension_parameter("max_bounding_box_z"));

  if (octree_depth == 8)
    octreeResolution = 0.079f;
  else if (octree_depth == 9)
    octreeResolution = 0.038f;
  else if (octree_depth == 10)
    octreeResolution = 0.019753f;
  else if (octree_depth == 11)
    octreeResolution = 0.0977f;
  else if (octree_depth == 12)
    octreeResolution = 0.0488f;
  else if (octree_depth == 13)
    octreeResolution = 0.0244f;
  else if (octree_depth == 14)
    octreeResolution = 0.0122f;

  // Define custom parameters for the compression
  float pointResolution = 0.001f;  // 1 mm precision for x, y, z coordinates
  bool doVoxelGridDownSampling = true;  // Enable voxel grid downsampling
  unsigned int iFrameRate = 0;          // Disable i-frames
  bool doColorEncoding = false;         // Disable color encoding
  unsigned int colorBitResolution = 0;  // No color bit resolution

  // Use pcl::PointXYZ for compression
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> pointcloud_compressor(
      pcl::io::MANUAL_CONFIGURATION,  // Custom configuration
      false,                          // showStatistics_arg = false
      pointResolution,                // Define coding precision (1 mm)
      octreeResolution,               // Set octree resolution to 5 cm
      doVoxelGridDownSampling,        // Enable voxel grid downsampling
      iFrameRate,                     // No i-frame rate (set to 0)
      doColorEncoding,                // Disable color encoding
      colorBitResolution              // Color bit resolution set to 0
  );

  pcl::PointCloud<AlfaPoint>::Ptr alfa_cloud(new pcl::PointCloud<AlfaPoint>);
  alfa_cloud = node->get_input_pointcloud();

  // Create a new pcl::PointCloud<pcl::PointXYZ> for the octree compressor
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int number_of_points = 0;

  // Convert each AlfaPoint to pcl::PointXYZ
  for (const auto& point : alfa_cloud->points) {
    if (is_point_inside_BB(point, bb)) {
      pcl_cloud->points.push_back(convertToPCLPoint(point));
      number_of_points++;
    }
  }

  std::stringstream compressedData;
  pointcloud_compressor.encodePointCloud(pcl_cloud, compressedData);

  // Check if decompression flag is set
  if (node->get_extension_parameter("decompression_flag") == 1) {
    // Decompress point cloud if decompression flag is set
    pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_compressor.decodePointCloud(compressedData, decompressedCloud);

    // Push decompressed points to output after converting to AlfaPoint
    for (const auto &pcl_point : decompressedCloud->points) {
      AlfaPoint alfa_point = convertToAlfaPoint(pcl_point);
      node->push_point_output_pointcloud(alfa_point);
    }
  } else {
    // Handle direct output of compressed data using memcpy for speed and efficiency
    std::string compressedString = compressedData.str();
    size_t compressedSize = compressedString.size();
    size_t code_index = 0;
    AlfaPoint point;

    // Process all complete AlfaPoints in the buffer
    while (code_index + sizeof(AlfaPoint) <= compressedSize) {
      // Copy the binary data into the AlfaPoint structure
      std::memcpy(&point, &compressedString[code_index], sizeof(AlfaPoint));

      // Push the deserialized point to the output point cloud
      node->push_point_output_pointcloud(point);

      // Move to the next chunk of compressed data
      code_index += sizeof(AlfaPoint);
    }

    // If there's leftover data at the end, copy what remains into the last point
    if (code_index < compressedSize) {
      std::memcpy(&point, &compressedString[code_index], compressedSize - code_index);
      node->push_point_output_pointcloud(point);  // Push the final, partially filled point
    }

    cout << compressedSize << endl;
  }
#endif
}

/**
 * @brief Post-processing function that publishes output point cloud and metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode *node) {
#ifdef EXT_HARDWARE
  while (1);  // Do nothing
#endif

  node->publish_pointcloud();
}

/**
 * @brief Entry point of the program.
 *
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Get subscriber topic from command line arguments
  std::string subscriber_topic = DEFAULT_TOPIC;
  if (argc > 1) {
    subscriber_topic = argv[1];
  }

#ifdef EXT_HARDWARE
  AlfaHardwareSupport hardware_support{false, true};
#else
  AlfaHardwareSupport hardware_support{false, false};
#endif

  AlfaConfiguration conf;
  AlfaExtensionParameter parameters[10];

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 1;
  conf.metrics_publishing_type = ALL_METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_INTENSITY;

  parameters[0].parameter_value = 12;
  parameters[0].parameter_name = "octree_depth";
  parameters[1].parameter_value = 100;
  parameters[1].parameter_name = "max_bounding_box_x";
  parameters[2].parameter_value = 100;
  parameters[2].parameter_name = "max_bounding_box_y";
  parameters[3].parameter_value = 100;
  parameters[3].parameter_name = "max_bounding_box_z";
  parameters[4].parameter_value = -100;
  parameters[4].parameter_name = "min_bounding_box_x";
  parameters[5].parameter_value = -100;
  parameters[5].parameter_name = "min_bounding_box_y";
  parameters[6].parameter_value = -100;
  parameters[6].parameter_name = "min_bounding_box_z";
  parameters[7].parameter_value = 0;
  parameters[7].parameter_name = "decompression_flag";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
