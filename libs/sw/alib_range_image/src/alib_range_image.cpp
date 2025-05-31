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

#include <cmath>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_fric_64"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0
#define HDL64

#ifdef HDL64
#define NUM_CHANNELS 64
#define RI_WIDTH 2048
#define RI_HEIGHT 128
#define MAX_ANGLE 5.5 * M_PI / 180
#define MIN_ANGLE -25.5 * M_PI / 180
#define MAX_RANGE 80
#define MIN_RANGE 0.5
#define DELTA_AZIMUTH 0.2 * M_PI / 180
#define DELTA_ELEVATION 0.47 * M_PI / 180
#define SENSOR 0
#endif

#ifdef HDL32
#define NUM_CHANNELS 32
#define RI_WIDTH 4096
#define RI_HEIGHT 32
#define MAX_ANGLE 11 * M_PI / 180
#define MIN_ANGLE -31 * M_PI / 180
#define MAX_RANGE 100
#define MIN_RANGE 0.5
#define SENSOR 1
#endif

#ifdef VLS128
#define NUM_CHANNELS 128
#define RI_WIDTH 2048
#define RI_HEIGHT 256
#define MAX_ANGLE 16 * M_PI / 180
#define MIN_ANGLE -26.5 * M_PI / 180
#define MAX_RANGE 100
#define MIN_RANGE 0.5
#define SENSOR 3
#endif

#ifdef VLP16
#define NUM_CHANNELS 16
#define RI_WIDTH 4096
#define RI_HEIGHT 16
#define MAX_ANGLE 15.1 * M_PI / 180
#define MIN_ANGLE -15.1 * M_PI / 180
#define MAX_RANGE 100
#define MIN_RANGE 1
#define SENSOR 2
#endif

uint16_t RangeImage[RI_WIDTH][RI_HEIGHT];
AlfaPoint OG_RangeImage[RI_WIDTH][RI_HEIGHT];
double frame_number = 0;

typedef struct SphericalPoint {
  float r;
  float theta;
  float phi;
} SphericalPoint;

SphericalPoint cartesian_to_spherical(AlfaPoint point);
double calculateMSE(AlfaPoint og_point, AlfaPoint re_point);

/**
 * @brief Handler function that copy input pointcloud into output pointcloud
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_SPHERICAL);
#else
  AlfaPoint point;
  while (node->get_point_input_pointcloud(point)) {
    SphericalPoint spherical_point = cartesian_to_spherical(point);
    if (spherical_point.phi >= MIN_ANGLE && spherical_point.phi <= MAX_ANGLE) {
      int x = round(0.5 * (1 + (spherical_point.theta / M_PI)) * (RI_WIDTH - 1));
      int y = round(((MAX_ANGLE - spherical_point.phi) / (MAX_ANGLE - MIN_ANGLE)) * (RI_HEIGHT - 1));
      uint16_t quantized = spherical_point.r * 100;
      OG_RangeImage[x][y] = point;
    }
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
  uint64_t *buffer = new uint64_t [size];
  node->read_ext_memory(1, sizeof(uint64_t) * size , buffer);
  for (int index = 0; index < size; index++) {
    uint16_t range = (buffer[index] >> 48) & 0xFFFF; // Extract bits 48-63
    uint32_t address = buffer[index] & 0xFFFFFFFF;
    uint16_t x = address % RI_WIDTH;
    uint16_t y = address / RI_WIDTH;
    RangeImage[x][y] = range;
  }

  #ifdef PUBLISH_RI
    for (int i = 0; i < RI_WIDTH; i++) {
      for (int j = 0; j < RI_HEIGHT; j++) {
        if (RangeImage[i][j] != 0) {
          AlfaPoint point;

          float theta = (2 * i - RI_WIDTH + 1) * M_PI / (RI_WIDTH - 1);
          float phi = MAX_ANGLE - (MAX_ANGLE - MIN_ANGLE) * (j) / (RI_HEIGHT - 1);
          float original_value = RangeImage[i][j] / 100.0;

          point.x = original_value * cos(phi) * sin(theta);
          point.y = original_value * cos(phi) * cos(theta);
          point.z = original_value * sin(phi);

          node->push_point_output_pointcloud(point);
        }
        RangeImage[i][j] = 0;
      }
    }
    node->publish_pointcloud();
    delete[] buffer;
  #endif
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

  parameters[0].parameter_value = SENSOR;
  parameters[0].parameter_name = "sensor";
  parameters[1].parameter_value = SPEED;
  parameters[1].parameter_name = "speed";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}


SphericalPoint cartesian_to_spherical(AlfaPoint point) {
    SphericalPoint spherical_point;

    spherical_point.r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    spherical_point.theta = atan2(point.x, point.y);
    spherical_point.phi = asin(point.z / spherical_point.r);

    return spherical_point;
}

double calculateMSE(AlfaPoint og_point, AlfaPoint re_point) {
    double dx = og_point.x - re_point.x;
    double dy = og_point.y - re_point.y;
    double dz = og_point.z - re_point.z;
    // std::cout << "dx: " << dx << " dy: " << dy << " dz: " << dz << std::endl;
    // std::cout << "og_x: " << og_point.x << " og_y: " << og_point.y << " og_z: " << og_point.z << std::endl;
    // std::cout << "re_x: " << re_point.x << " re_y: " << re_point.y << " re_z: " << re_point.z << std::endl;

    return sqrt(dx * dx + dy * dy + dz * dz);
}
