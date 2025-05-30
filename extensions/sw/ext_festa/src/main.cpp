/*
 * Copyright 2025 ALFA Project. All rights reserved.
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

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "alib_metrics.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_festa"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0
#define METRICS ALL_METRICS  // FULL_PROCESSING, HANDLER_PROCESSING
#define GRID_WIDTH_X 512     // cells
#define GRID_WIDTH_Y 256     // cells
#define CELL_SIZE 0.5

struct Cell {  // Structure declaration
  float minz = 100000;
  float maxz = 0;
};  // Structure variable

/**
 * @brief Handler function that applies a distance filter to the received point
 * cloud.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN, node->get_input_pointcloud());
#else
  // Parameters
  float zeta = node->get_extension_parameter("zeta");
  float epsilon = node->get_extension_parameter("epsilon");
  float delta = node->get_extension_parameter("delta");
  int output_type = static_cast<int>(node->get_extension_parameter("output_type"));

  // Grid setup
  int num_cells = (GRID_WIDTH_X * GRID_WIDTH_Y) + 1;

  vector<Cell> grid(num_cells);

  // First pass: update min/max z per cell
  for (const auto& pt : node->get_input_pointcloud()->points) {
    float x_norm = pt.x + (GRID_WIDTH_X * CELL_SIZE) / 2;
    float y_norm = (GRID_WIDTH_Y * CELL_SIZE) / 2 - pt.y;

    int cell_x = static_cast<int>(x_norm / CELL_SIZE);
    int cell_y = static_cast<int>(y_norm / CELL_SIZE);

    if (cell_x < 0 || cell_x >= GRID_WIDTH_X || cell_y < 0 || cell_y >= GRID_WIDTH_Y) continue;

    int index = cell_x + cell_y * GRID_WIDTH_X;
    if (pt.z < grid[index].minz) grid[index].minz = pt.z;
    if (pt.z > grid[index].maxz) grid[index].maxz = pt.z;
  }

  // Second pass: classify and collect points
  std::vector<AlfaPoint> ground_points;
  std::vector<AlfaPoint> non_ground_points;

  int id = 0;
  for (const auto& pt_in : node->get_input_pointcloud()->points) {
    float x_norm = pt_in.x + (GRID_WIDTH_X * CELL_SIZE) / 2;
    float y_norm = (GRID_WIDTH_Y * CELL_SIZE) / 2 - pt_in.y;

    int cell_x = static_cast<int>(x_norm / CELL_SIZE);
    int cell_y = static_cast<int>(y_norm / CELL_SIZE);

    AlfaPoint pt = pt_in;
    int label = ALFA_LABEL_NO_GROUND;

    if (cell_x >= 0 && cell_x < GRID_WIDTH_X && cell_y >= 0 && cell_y < GRID_WIDTH_Y) {
      int index = cell_x + cell_y * GRID_WIDTH_X;

      float minz = grid[index].minz;
      float maxz = grid[index].maxz;
      float z_sub = maxz - minz;
      float z_th = z_sub / 5.0f;

      if (minz < zeta) {
        if (z_sub >= epsilon) {
          if (pt.z < (minz + delta)) label = ALFA_LABEL_GROUND;
        } else {
          if (pt.z < (minz + z_th)) label = ALFA_LABEL_GROUND;
        }
      }
    }

    pt.custom_field = label;

    if (label == ALFA_LABEL_GROUND)
      ground_points.push_back(pt);
    else
      non_ground_points.push_back(pt);

    ++id;
  }

  // Output selection
  if (output_type == 0) {
    for (const auto& p : ground_points) node->push_point_output_pointcloud(p);
    for (const auto& p : non_ground_points) node->push_point_output_pointcloud(p);
  } else if (output_type == 1) {
    for (const auto& p : ground_points) node->push_point_output_pointcloud(p);
  } else if (output_type == 2) {
    for (const auto& p : non_ground_points) node->push_point_output_pointcloud(p);
  }

#endif
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->load_pointcloud(LOAD_STORE_CARTESIAN, node->get_output_pointcloud());
#endif

  AlfaGroundSegmentationMetrics metrics = evaluate_ground_segmentation_method(node, GROUND_LABELS);

  node->publish_pointcloud();
  node->set_debug_point(0, metrics.basic_metrics.number_input_points, "Number of input points");
  node->set_debug_point(1, metrics.basic_metrics.number_output_points, "Number of output points");
  node->set_debug_point(2, metrics.true_positive, "True positive");
  node->set_debug_point(3, metrics.false_positive, "False positive");
  node->set_debug_point(4, metrics.true_negative, "True negative");
  node->set_debug_point(5, metrics.false_negative, "False negative");
  node->set_debug_point(6, metrics.true_positive_rate, "True positive rate");
  node->set_debug_point(7, metrics.true_negative_rate, "True negative rate");
  node->set_debug_point(8, metrics.false_positive_rate, "False positive rate");
  node->set_debug_point(9, metrics.false_negative_rate, "False negative rate");
  node->set_debug_point(10, metrics.positive_predictive_value, "Positive predictive value");
  node->set_debug_point(11, metrics.negative_predictive_value, "Negative predictive value");
  node->set_debug_point(12, metrics.f1_score, "F1 score");
  node->set_debug_point(13, metrics.accuracy, "Accuracy");
  node->set_debug_point(14, metrics.IoUg, "IoUg");
  node->set_debug_point(15, metrics.number_input_ground_points,
                        "Number of ground points in the input point cloud");
  node->set_debug_point(16, metrics.number_output_ground_points,
                        "Number of ground points in the output point cloud");
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

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 18;
  conf.metrics_publishing_type = METRICS;  // FULL_PROCESSING, HANDLER_PROCESSING
  conf.custom_field_conversion_type = CUSTOM_FIELD_LABEL;

  vector<AlfaExtensionParameter> parameters(4);

  parameters[0].parameter_name = "zeta";  // threshold
  parameters[0].parameter_value = -1;     //[m]

  parameters[1].parameter_name = "epsilon";  // threshold
  parameters[1].parameter_value = 0.5;       //[m]

  parameters[2].parameter_name = "delta";  // threshold
  parameters[2].parameter_value = 0.15;    //[m]

  parameters[3].parameter_name = "output_type";  // threshold
  parameters[3].parameter_value = 0;             // 0: labeled as ground and non-ground
                                                 // 1: only ground
                                                 // 2: only non-ground

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
