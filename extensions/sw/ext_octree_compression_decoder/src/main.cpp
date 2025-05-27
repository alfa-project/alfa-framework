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

#include <bitset>
#include <cmath>
#include <vector>

#include "alfa_node.hpp"
#include "alib_compression.hpp"
#include "alib_octree.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_octree_compression_decoder"
#define DEFAULT_TOPIC "/ext_octree_compression_encoder_pointcloud"

#define NODE_ID 0
#define POINTCLOUD_ID 0

/**
 * @brief Handler function that decodes the bitstream sent by
 * ext_octree_compression_encoder_pointcloud topic and publishes that decoded point cloud.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
  AlfaBB bb(node->get_extension_parameter("min_bounding_box_x"),
            node->get_extension_parameter("min_bounding_box_y"),
            node->get_extension_parameter("min_bounding_box_z"),
            node->get_extension_parameter("max_bounding_box_x"),
            node->get_extension_parameter("max_bounding_box_y"),
            node->get_extension_parameter("max_bounding_box_z"));

  // Convert input pointcloud into bitstream
  vector<unsigned char> code;
  auto compressed_code = convert_AlfaPoint_vector_to_code(node->get_input_pointcloud_as_vector());

  AlfaOctree octree(bb, (int)node->get_extension_parameter("octree_depth"), false);

  if (node->get_extension_parameter("compression_algorithm") == 1) {
    code = alib_rle_decode(compressed_code);
  } else if (node->get_extension_parameter("compression_algorithm") == 2) {
    code = alib_lz4_decode(compressed_code, 4);
  } else if (node->get_extension_parameter("compression_algorithm") == 3) {
    code = alib_lz77_decode(compressed_code);
  } else if (node->get_extension_parameter("compression_algorithm") == 4) {
    code = alib_huffman_decode(compressed_code);
  } else if (node->get_extension_parameter("compression_algorithm") == 5) {
    code = alib_huffman_s_decode(compressed_code);
  } else {
    code = compressed_code;
  }

  octree.init_octree_from_occupation_code_DFS(code, bb);
  octree.convert_to_pointcloud(node->get_output_pointcloud());
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode *node) { node->publish_pointcloud(); }

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

  // Decoder does not use hardware
  AlfaHardwareSupport hardware_support{false, false};

  AlfaConfiguration conf;

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 1;
  conf.metrics_publishing_type = ALL_METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_USER;

  // Create the parameters
  std::vector<AlfaExtensionParameter> parameters(8);

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
  parameters[7].parameter_name = "compression_algorithm";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
