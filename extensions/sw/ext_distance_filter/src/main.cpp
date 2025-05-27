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

#include <cmath>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_distance_filter"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0

/**
 * @brief Handler function that applies a distance filter to the received point
 * cloud.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN);
#else
  // Simple example code aplying a distance filter to the pointcloud
  // Get PCL pointcloud
  AlfaPoint point;

  //
  float min_distance = node->get_extension_parameter("min_distance");
  float max_distance = node->get_extension_parameter("max_distance");
  float number_outside_points = 0;

  while (node->get_point_input_pointcloud(point)) {
    // Calculate distance to point
    float distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    // If distance falls between the set parameters, add to output pointcloud
    if (distance > min_distance && distance < max_distance) {
      node->push_point_output_pointcloud(point);
    } else
      number_outside_points++;
  }

  node->set_debug_point(0, number_outside_points, "Number of points outside the distance range");
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
  node->load_pointcloud(LOAD_STORE_CARTESIAN);
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

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 1;
  conf.metrics_publishing_type = ALL_METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_FILTER;

  // Create the parameters
  std::vector<AlfaExtensionParameter> parameters(2);

  parameters[1].parameter_value = 5;
  parameters[1].parameter_name = "min_distance";
  parameters[0].parameter_value = 20;
  parameters[0].parameter_name = "max_distance";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
