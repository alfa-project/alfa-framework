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

#include "alib_compression.hpp"

#include "alfa_structs.hpp"

void print_frequencies(const std::vector<unsigned char>& input) {
  std::unordered_map<unsigned char, int> frequency_map;
  for (unsigned char c : input) {
    frequency_map[c]++;
  }

  for (const auto& pair : frequency_map) {
    std::cout << "code " << static_cast<int>(pair.first) << " -> " << pair.second << " times"
              << std::endl;
  }
}

void write_binary_file(const std::string& file_path,
                       const std::vector<unsigned char>& data) {
  // Open the file in binary mode
  std::ofstream file(file_path, std::ios::binary);
  if (!file) {
    std::cerr << "Error: Could not open file " << file_path << " for writing.\n";
    return;
  }

  // Write the data to the file
  file.write(reinterpret_cast<const char*>(data.data()), data.size());

  // Close the file
  file.close();

  // Check for successful write
  if (file) {
    std::cout << "Data successfully written to " << file_path << "\n";
  } else {
    std::cerr << "Error: Writing to file " << file_path << " failed.\n";
  }
}

void print_vector_binary(const std::vector<unsigned char>& input) {
  for (unsigned char c : input) {
    std::cout << std::bitset<8>(c) << ' ';
  }
  std::cout << std::endl;
}

void convert_code_pointcloud(const std::vector<unsigned char>& code,
                             pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  pointcloud->clear();
  pointcloud->height = 1;
  pointcloud->width = 0;
  pointcloud->is_dense = false;

  size_t code_index = 0;
  size_t code_bytes_last_point = code.size() % sizeof(AlfaPoint);
  AlfaPoint point_temp;

  // Store the number of bytes used in last AlfaPoint
  point_temp.x = static_cast<float>(code_bytes_last_point);
  point_temp.y = static_cast<float>(code_bytes_last_point);
  point_temp.z = static_cast<float>(code_bytes_last_point);
  pointcloud->points.push_back(point_temp);
  ++pointcloud->width;

  // Store the remaining bytes in the pointcloud
  while (code_index + sizeof(AlfaPoint) <= code.size()) {
    std::memcpy(&point_temp, &code[code_index], sizeof(AlfaPoint));
    pointcloud->points.push_back(point_temp);
    ++pointcloud->width;
    code_index += sizeof(AlfaPoint);
  }

  if (code_index < code.size()) {
    std::memset(&point_temp, 0, sizeof(AlfaPoint));
    std::memcpy(&point_temp, &code[code_index], code.size() - code_index);
    pointcloud->points.push_back(point_temp);
    ++pointcloud->width;
  }
}

void convert_pointcloud_code(const pcl::PointCloud<AlfaPoint>::Ptr pointcloud,
                             std::vector<unsigned char>& code) {
  code.clear();

  if (pointcloud->empty()) return;

  // Read metadata from the first point
  const AlfaPoint& point_temp = pointcloud->points[0];
  size_t code_bytes_last_point = static_cast<size_t>(point_temp.x);
  bool has_partial_point = (code_bytes_last_point > 0);

  // Total data points = size - 1 (excluding metadata)
  size_t total_data_points = pointcloud->points.size() - 1;
  size_t num_full_points =
      has_partial_point ? total_data_points - 1 : total_data_points;

  // Allocate space for full points + partial (if any)
  size_t total_code_size =
      num_full_points * sizeof(AlfaPoint) + code_bytes_last_point;
  code.resize(total_code_size);

  // Copy full points
  for (size_t i = 0; i < num_full_points; ++i) {
    const AlfaPoint& pt = pointcloud->points[i + 1];  // skip metadata
    std::memcpy(&code[i * sizeof(AlfaPoint)], &pt, sizeof(AlfaPoint));
  }

  // Copy partial point if needed
  if (has_partial_point) {
    const AlfaPoint& partial_pt = pointcloud->points.back();  // last point
    std::memcpy(&code[num_full_points * sizeof(AlfaPoint)], &partial_pt,
                code_bytes_last_point);
  }
}
