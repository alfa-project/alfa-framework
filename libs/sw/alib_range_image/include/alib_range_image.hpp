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

#ifndef ALIB_RANGEIMAGE_H
#define ALIB_RANGEIMAGE_H

#include <cmath>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "rclcpp/rclcpp.hpp"


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

typedef struct SphericalPoint {
  float r;
  float theta;
  float phi;
} SphericalPoint;


class AlfaRI {
 public:
  AlfaRI();
  ~AlfaRI();

  void convert_to_range_image(pcl::PointCloud<AlfaPoint>::Ptr pointcloud, uint16_t (&RangeImage)[RI_WIDTH][RI_HEIGHT]);
  pcl::PointCloud<AlfaPoint>::Ptr convert_to_pointcloud(uint16_t (&RangeImage)[RI_WIDTH][RI_HEIGHT]);

 private:
  SphericalPoint spherical_point;
  SphericalPoint cartesian_to_spherical(AlfaPoint point);
  AlfaPoint point;
  std::uint32_t point_counter;
  pcl::PointCloud<AlfaPoint>::Ptr output_pointcloud;
};



#endif  // ALIB_OCTREE_H
