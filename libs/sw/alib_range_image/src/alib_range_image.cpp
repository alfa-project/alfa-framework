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

#include "alib_range_image.hpp"

AlfaRI::AlfaRI() {
  output_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);
}

AlfaRI::~AlfaRI() {
}

void AlfaRI::convert_to_range_image(pcl::PointCloud<AlfaPoint>::Ptr pointcloud, uint16_t (&RangeImage)[RI_WIDTH][RI_HEIGHT]) {
  point_counter = 0;
  std::uint32_t size = pointcloud->size();
  while (point_counter < size) {
    point = (*pointcloud)[point_counter++];
    SphericalPoint spherical_point = cartesian_to_spherical(point);
    if (spherical_point.phi >= MIN_ANGLE && spherical_point.phi <= MAX_ANGLE) {
      int x = round(0.5 * (1 + (spherical_point.theta / M_PI)) * (RI_WIDTH - 1));
      int y = round(((MAX_ANGLE - spherical_point.phi) / (MAX_ANGLE - MIN_ANGLE)) * (RI_HEIGHT - 1));
      uint16_t quantized = spherical_point.r * 100;
      RangeImage[x][y] = quantized;
    }
  }
}

pcl::PointCloud<AlfaPoint>::Ptr AlfaRI::convert_to_pointcloud(uint16_t (&RangeImage)[RI_WIDTH][RI_HEIGHT]) {

  for (int i = 0; i < RI_WIDTH; i++) {
    for (int j = 0; j < RI_HEIGHT; j++) {
      if (RangeImage[i][j] != 0) {
        float theta = (2 * i - RI_WIDTH + 1) * M_PI / (RI_WIDTH - 1);
        float phi = MAX_ANGLE - (MAX_ANGLE - MIN_ANGLE) * (j) / (RI_HEIGHT - 1);
        float original_value = RangeImage[i][j] / 100.0;

        point.x = original_value * cos(phi) * sin(theta);
        point.y = original_value * cos(phi) * cos(theta);
        point.z = original_value * sin(phi);

        output_pointcloud->push_back(point);
      }
    }
  }

  return output_pointcloud;
}

SphericalPoint AlfaRI::cartesian_to_spherical(AlfaPoint point) {
    SphericalPoint spherical_point;

    spherical_point.r = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    spherical_point.theta = atan2(point.x, point.y);
    spherical_point.phi = asin(point.z / spherical_point.r);

    return spherical_point;
}

