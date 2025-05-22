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

// Convert raw byte code into a vector of AlfaPoint
std::vector<AlfaPoint> convert_code_to_AlfaPoint_vector(
    const std::vector<unsigned char>& code) {
  std::vector<AlfaPoint> pointcloud;
  if (code.empty()) return pointcloud;

  size_t total_bytes = code.size();
  size_t full_points = total_bytes / sizeof(AlfaPoint);
  size_t partial_bytes = total_bytes % sizeof(AlfaPoint);
  size_t code_index = 0;

  // Create header point: store total byte size in custom_field
  AlfaPoint header;
  std::memset(&header, 0, sizeof(AlfaPoint));

  header.custom_field = static_cast<std::uint32_t>(total_bytes);
  pointcloud.push_back(header);

  // Copy full AlfaPoints
  AlfaPoint temp;
  for (size_t i = 0; i < full_points; ++i) {
    std::memcpy(&temp, &code[code_index], sizeof(AlfaPoint));
    pointcloud.push_back(temp);
    code_index += sizeof(AlfaPoint);
  }

  // Copy last partial block if needed
  if (partial_bytes > 0) {
    std::memset(&temp, 0, sizeof(AlfaPoint));
    std::memcpy(&temp, &code[code_index], partial_bytes);
    pointcloud.push_back(temp);
  }

  return pointcloud;
}

// Convert a vector of AlfaPoint into a raw byte code
std::vector<unsigned char> convert_AlfaPoint_vector_to_code(
    const std::vector<AlfaPoint>& pointcloud) {
  std::vector<unsigned char> code;

  if (pointcloud.empty()) return code;

  const AlfaPoint& header = pointcloud[0];
  uint32_t total_code_bytes = header.custom_field;

  size_t full_points = total_code_bytes / sizeof(AlfaPoint);
  size_t partial_bytes = total_code_bytes % sizeof(AlfaPoint);
  size_t total_data_points = full_points + (partial_bytes > 0 ? 1 : 0);

  // Sanity check
  if (pointcloud.size() < 1 + total_data_points) return code;

  code.resize(total_code_bytes);

  // Copy full points
  for (size_t i = 0; i < full_points; ++i) {
    const AlfaPoint& pt = pointcloud[i + 1];
    std::memcpy(&code[i * sizeof(AlfaPoint)], &pt, sizeof(AlfaPoint));
  }

  // Copy partial point if needed
  if (partial_bytes > 0) {
    const AlfaPoint& last = pointcloud[1 + full_points];
    std::memcpy(&code[full_points * sizeof(AlfaPoint)], &last, partial_bytes);
  }

  return code;
}
