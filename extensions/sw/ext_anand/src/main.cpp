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

#define NODE_NAME "ext_anand"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0
#define METRICS ALL_METRICS  // FULL_PROCESSING, HANDLER_PROCESSING

#define csize 0.5

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

  float zeta = node->get_extension_parameter("zeta");
  float epsilon = node->get_extension_parameter("epsilon");
  float delta = node->get_extension_parameter("delta");
  int output_type = static_cast<int>(node->get_extension_parameter("output_type"));

  double x_min = 0, x_max = 0, y_min = 0, y_max = 0;

  // Initialize min and max values
  for (const auto &point : node->get_input_pointcloud()->points) {
    if (point.x < x_min) x_min = point.x;
    if (point.x > x_max) x_max = point.x;
    if (point.y < y_min) y_min = point.y;
    if (point.y > y_max) y_max = point.y;
  }

  // Calculate the number of cells in the grid
  int num_cells_x = std::ceil((x_max - x_min) / csize);
  int num_cells_y = std::ceil((y_max - y_min) / csize);

  // Initialize the grid with empty vectors
  std::vector<std::vector<std::vector<int> > > grid(num_cells_x,
                                                    std::vector<std::vector<int> >(num_cells_y));

  int t = 0;
  // Fill the grid with point indices
  for (const auto &point : node->get_input_pointcloud()->points) {
    int cell_x = std::floor((point.x - x_min) / csize);
    int cell_y = std::floor((point.y - y_min) / csize);

    if (cell_x == num_cells_x) cell_x -= 1;
    if (cell_y == num_cells_y) cell_y -= 1;

    grid[cell_x][cell_y].push_back(t++);
  }

  std::vector<AlfaPoint> ground_points;
  std::vector<AlfaPoint> non_ground_points;

  // Process each cell in the grid
  for (int j = 0; j < num_cells_y; j++) {
    for (int i = 0; i < num_cells_x; i++) {
      if (!grid[i][j].empty()) {
        double z_min = 10;
        double z_max = 0;

        // Find min and max z values in the cell
        for (const auto index : grid[i][j]) {
          AlfaPoint point = node->get_point_input_pointcloud(index);
          if (point.z < z_min) z_min = point.z;
          if (point.z > z_max) z_max = point.z;
        }

        double z_sub = z_max - z_min;
        double z_th = z_sub / 5;

        // Calculate the zeta value based on the cell size
        for (const auto index : grid[i][j]) {
          AlfaPoint point = node->get_point_input_pointcloud(index);
          int label = ALFA_LABEL_NO_GROUND;

          if (z_min < zeta) {
            if (z_sub >= epsilon) {
              if (point.z < (z_min + delta)) label = ALFA_LABEL_GROUND;
            } else {
              if (point.z < (z_min + z_th)) label = ALFA_LABEL_GROUND;
            }
          }

          point.custom_field = label;  // Set label into the point

          // Store the point in the appropriate vector
          if (label == ALFA_LABEL_GROUND)
            ground_points.push_back(point);
          else
            non_ground_points.push_back(point);
        }
      }
    }
  }

  // Add points to the output point cloud based on output_type
  if (output_type == 0) {
    for (const auto &p : ground_points) node->push_point_output_pointcloud(p);
    for (const auto &p : non_ground_points) node->push_point_output_pointcloud(p);
  } else if (output_type == 1) {
    for (const auto &p : ground_points) node->push_point_output_pointcloud(p);
  } else if (output_type == 2) {
    for (const auto &p : non_ground_points) node->push_point_output_pointcloud(p);
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
  // outside_points = 0;
#ifdef EXT_HARDWARE
  node->load_pointcloud(LOAD_STORE_CARTESIAN, node->get_output_pointcloud());
  ground_points = node->get_debug_point(0);
  outside_points = node->get_debug_point(1);
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
  conf.metrics_publishing_type = METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_LABEL;

  std::vector<AlfaExtensionParameter> parameters;
  parameters.resize(4);  // Adjust the size of the parameters vector

  parameters[0].parameter_name = "zeta";  // threshold
  parameters[0].parameter_value = -1.0;   //[m]

  parameters[1].parameter_name = "epsilon";  // threshold
  parameters[1].parameter_value = 0.5;       //[m] 0.5

  parameters[2].parameter_name = "delta";  // threshold
  parameters[2].parameter_value = 0.12;    //[m] 0.12

  parameters[3].parameter_name = "output_type";
  parameters[3].parameter_value = 0;  // 0: labeled as ground and non-ground
                                      // 1: only ground
                                      // 2: only non-ground

  // AnandMetrics.reset(
  //     new Metrics<PointType>(2, "Removed points", "Points outside of grid"));

  // rclcpp::on_shutdown(&callback_shutdown);

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}