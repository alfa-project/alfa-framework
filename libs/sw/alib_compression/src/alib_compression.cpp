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

void calculateAndPrintFrequencies(const std::vector<unsigned char>& input) {
  std::unordered_map<unsigned char, int> frequency_map;
  for (unsigned char c : input) {
    frequency_map[c]++;
  }

  for (const auto& pair : frequency_map) {
    std::cout << "code " << static_cast<int>(pair.first) << " -> " << pair.second << " times"
              << std::endl;
  }
}

void writeBinaryToFile(const std::string& file_path, const std::vector<unsigned char>& data) {
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

void printVectorAsBinary(const std::vector<unsigned char>& input) {
  for (unsigned char c : input) {
    std::cout << std::bitset<8>(c) << ' ';
  }
  std::cout << std::endl;
}
