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

#include <algorithm>
#include <bitset>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "alib_compression.hpp"

std::vector<unsigned char> alib_huffman_s_encode(const std::vector<unsigned char>& input) {
  std::vector<int> frequency(256, 0);

  // Calculate frequency of each character
  for (unsigned char c : input) {
    frequency[c]++;
  }

  // Sort characters based on frequency and value
  std::vector<unsigned char> rank_map(256, 0);
  std::vector<unsigned char> sorted_chars(256);
  for (int i = 0; i < 256; ++i) {
    sorted_chars[i] = static_cast<unsigned char>(i);
  }
  std::sort(sorted_chars.begin(), sorted_chars.end(),
            [&frequency](unsigned char a, unsigned char b) {
              if (frequency[a] == frequency[b]) {
                return a < b;  // Break ties by value
              }
              return frequency[a] > frequency[b];
            });

  for (int i = 0; i < 256; ++i) {
    rank_map[sorted_chars[i]] = static_cast<unsigned char>(i);
  }

  // Create encoded data header
  std::vector<unsigned char> encoded_data;

  // Reserve space for last_byte_bits at the start
  encoded_data.push_back(0);  // Placeholder for last_byte_bits
  encoded_data.insert(encoded_data.end(), rank_map.begin(), rank_map.end());

  int bit_position = 0;
  unsigned char current_byte = 0;

  // Encode each character in the input
  for (unsigned char c : input) {
    unsigned char index = rank_map[c];

    std::string code;
    if (index < 4) {
      code = "0" + std::bitset<2>(index).to_string();
    } else if (index < 12) {
      code = "10" + std::bitset<3>(index - 4).to_string();
    } else if (index < 28) {
      code = "110" + std::bitset<4>(index - 12).to_string();
    } else if (index < 92) {
      code = "1110" + std::bitset<6>(index - 28).to_string();
    } else if (index < 256) {
      code = "11110" + std::bitset<8>(index - 92).to_string();
    }

    // Append code bits to the current byte
    for (char bit : code) {
      if (bit == '1') {
        current_byte |= (1 << (7 - bit_position));
      }
      bit_position++;
      if (bit_position == 8) {
        encoded_data.push_back(current_byte);
        current_byte = 0;
        bit_position = 0;
      }
    }
  }

  // Append remaining bits in the last byte if needed
  if (bit_position > 0) {
    encoded_data.push_back(current_byte);
  }

  // Store the number of valid bits in the last byte at the start
  encoded_data[0] = static_cast<unsigned char>(bit_position);

  return encoded_data;
}

std::vector<unsigned char> alib_huffman_s_decode(const std::vector<unsigned char>& encoded_data) {
  // Extract last_byte_bits from the first byte
  size_t last_byte_bits = encoded_data[0];

  // Extract rank mapping from the encoded data
  std::vector<unsigned char> rank_map(encoded_data.begin() + 1, encoded_data.begin() + 257);

  // Convert remaining data into a bit stream
  std::string bit_stream;
  for (size_t i = 257; i < encoded_data.size(); ++i) {
    bit_stream += std::bitset<8>(encoded_data[i]).to_string();
  }

  std::vector<unsigned char> decoded_data;
  size_t bit_index = 0;

  // Decode each character from the bit stream
  while (bit_index < bit_stream.size() - (8 - last_byte_bits) % 8) {
    int index = -1;

    // Determine the tier and decode index based on bit prefix
    if (bit_stream[bit_index] == '0') {
      index = std::stoi(bit_stream.substr(bit_index + 1, 2), nullptr, 2);
      bit_index += 3;
    } else if (bit_stream.substr(bit_index, 2) == "10") {
      index = 4 + std::stoi(bit_stream.substr(bit_index + 2, 3), nullptr, 2);
      bit_index += 5;
    } else if (bit_stream.substr(bit_index, 3) == "110") {
      index = 12 + std::stoi(bit_stream.substr(bit_index + 3, 4), nullptr, 2);
      bit_index += 7;
    } else if (bit_stream.substr(bit_index, 4) == "1110") {
      index = 28 + std::stoi(bit_stream.substr(bit_index + 4, 6), nullptr, 2);
      bit_index += 10;
    } else if (bit_stream.substr(bit_index, 5) == "11110") {
      index = 92 + std::stoi(bit_stream.substr(bit_index + 5, 8), nullptr, 2);
      bit_index += 13;
    } else {
      std::cerr << "Error: Invalid tier or bit sequence detected at bit index " << bit_index
                << std::endl;
      break;
    }

    if (index < 0 || index >= 256) {
      std::cerr << "Error: Invalid index detected. Index: " << index << std::endl;
      break;
    }

    // Decode the character and append to the output
    unsigned char decoded_char = static_cast<unsigned char>(
        std::find(rank_map.begin(), rank_map.end(), index) - rank_map.begin());
    decoded_data.push_back(decoded_char);
  }

  return decoded_data;
}
