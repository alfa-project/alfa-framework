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

#ifndef ALFA_STRUCTS_H
#define ALFA_STRUCTS_H

#include <pcl/point_types.h>

#include <chrono>
#include <string>

#include "alfa_msg/msg/alfa_metrics.hpp"

using namespace std;

struct AlfaPoint {
  float x, y, z;
  std::uint32_t custom_field;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct AlfaHardwareSupport {
  bool hardware_driver;
  bool hardware_extension;
};

struct AlfaExtensionParameter {
  string parameter_name;
  double parameter_value;
};

struct alfa_ext_ioctl_data {
  unsigned int extension_id;  // ID of the extension region
  unsigned int offset;        // Offset of the register
  unsigned int value;         // Value to write or the read value
};

struct AlfaPointcloud {
  std::uint32_t size;
  std::uint64_t *ptr;
};

struct AlfaConfiguration {
  string subscriber_topic;
  string node_name;
  std::uint32_t pointcloud_id;
  std::uint32_t extension_id;
  AlfaHardwareSupport hardware_support;
  std::uint32_t latency;
  std::uint32_t metrics_publishing_type;
  std::uint32_t custom_field_conversion_type;
  std::uint32_t number_of_debug_points;
};

struct AlfaMetric {
  std::chrono::_V2::system_clock::time_point start, stop;
  alfa_msg::msg::MetricMessage message;
};

#endif  // ALFA_STRUCTS_H