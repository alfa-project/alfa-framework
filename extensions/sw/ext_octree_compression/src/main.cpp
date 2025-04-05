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

#include <bitset>
#include <cmath>
#include <vector>

#include "alfa_node.hpp"
#include "alib_compression.hpp"
#include "alib_octree.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_octree_compression"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0

/**
 * @brief Handler function that copy input pointcloud into output pointcloud
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN);
#else

  AlfaBB bb(node->get_extension_parameter("min_bounding_box_x"),
            node->get_extension_parameter("min_bounding_box_y"),
            node->get_extension_parameter("min_bounding_box_z"),
            node->get_extension_parameter("max_bounding_box_x"),
            node->get_extension_parameter("max_bounding_box_y"),
            node->get_extension_parameter("max_bounding_box_z"));

  AlfaOctree octree1(bb, (int)node->get_extension_parameter("octree_depth"),
                     false);
  AlfaPoint point;

  octree1.insert_pointcloud(node->get_input_pointcloud());
  auto original_size = octree1.get_number_of_points() * sizeof(AlfaPoint);

  vector<unsigned char> code = octree1.get_occupation_code_DFS();

  if (node->get_extension_parameter("decompression_flag") == 1) {
    AlfaOctree octree2(bb, node->get_extension_parameter("octree_depth"),
                       false);
    octree2.init_octree_from_occupation_code_DFS(code, bb);
    auto pointcloud = octree2.convert_to_pointcloud();
    for (auto point : pointcloud) node->push_point_output_pointcloud(point);

  } else {
    auto compressed_code_RLE = alib_rle_encode(code);
    auto compressed_code_LZ4 = alib_lz4_encode(code, 4, 65535, 15, 19);
    auto compresed_code_lz77 = alib_lz77_encode(code, 4096, 18);
    auto compressed_code_Huffman = alib_huffman_encode(code);
    auto compressed_code_fog = alib_huffman_s_encode(code);

    cout << original_size << " " << code.size() << " "
         << compressed_code_RLE.size() << " " << compressed_code_LZ4.size()
         << " " << compresed_code_lz77.size() << " "
         << compressed_code_Huffman.size() << " " << compressed_code_fog.size()
         << endl;
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

  struct bs_code {
    uint64_t size;
    std::vector<char> code;
  } buffer;

  node->read_ext_memory(0, sizeof(buffer.size), &(buffer.size));
  buffer.code.resize(buffer.size);
  node->read_ext_memory(1, buffer.size * sizeof(char), buffer.code.data());

  if (node->get_extension_parameter("decompression_flag") == 1) {
    AlfaBB bb(node->get_extension_parameter("min_bounding_box_x"),
              node->get_extension_parameter("min_bounding_box_y"),
              node->get_extension_parameter("min_bounding_box_z"),
              node->get_extension_parameter("max_bounding_box_x"),
              node->get_extension_parameter("max_bounding_box_y"),
              node->get_extension_parameter("max_bounding_box_z"));

    AlfaOctree octree1(bb, node->get_extension_parameter("octree_depth"),
                       false);

    octree1.init_octree_from_occupation_code_DFS(buffer.code, bb);
    auto pointcloud = octree1.convert_to_pointcloud();
    for (auto point : pointcloud) node->push_point_output_pointcloud(point);

  } else {
    size_t code_index = 0;
    AlfaPoint point;

    while (code_index + sizeof(AlfaPoint) < buffer.code.size()) {
      std::memcpy(&point, &buffer.code[code_index], sizeof(AlfaPoint));
      node->push_point_output_pointcloud(point);
      code_index += sizeof(AlfaPoint);
    }

    if (code_index < buffer.code.size()) {
      std::memcpy(&point, &buffer.code[code_index],
                  sizeof(buffer.code.size() - code_index));
      node->push_point_output_pointcloud(point);
    }
  }

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
  rclcpp::spin(
      std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
