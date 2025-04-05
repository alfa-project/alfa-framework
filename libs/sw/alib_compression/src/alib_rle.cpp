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

#include "alib_compression.hpp"

// RLE encoding function
std::vector<unsigned char> alib_rle_encode(
    const std::vector<unsigned char>& input) {
  std::vector<unsigned char> encoded;
  size_t bit_count = input.size() * 8;  // Total number of bits
  size_t bit_index = 0;

  while (bit_index < bit_count) {
    // Determine the current bit value (0 or 1)
    uint8_t byte = input[bit_index / 8];
    uint8_t bit_value = (byte >> (7 - (bit_index % 8))) & 1;

    uint8_t run_length = 0;

    // Count consecutive bits up to 63 (Long Run max)
    while (bit_index < bit_count) {
      byte = input[bit_index / 8];
      uint8_t current_bit = (byte >> (7 - (bit_index % 8))) & 1;

      if (current_bit != bit_value || run_length == 63) {
        break;
      }
      run_length++;
      bit_index++;
    }

    if (run_length <= 3) {
      // Short Run Encoding: 1 bit (indicator) + 2 bits (length) + 1 bit (value)
      uint8_t short_encoded = (0 << 7) | (run_length << 1) | bit_value;

      // If there's already a Short Run in the last byte, combine it
      if (!encoded.empty() && (encoded.back() & 0xF0) == 0) {
        encoded.back() = (encoded.back() << 4) | short_encoded;
      } else {
        encoded.push_back(short_encoded);
      }
    } else {
      // Long Run Encoding: 1 bit (indicator) + 6 bits (length) + 1 bit (value)
      uint8_t long_encoded = (1 << 7) | (run_length << 1) | bit_value;
      encoded.push_back(long_encoded);
    }
  }

  return encoded;
}

// RLE decoding function
std::vector<unsigned char> alib_rle_decode(
    const std::vector<unsigned char>& encoded) {
  std::vector<unsigned char> decoded;
  uint8_t current_byte = 0;
  uint8_t bit_position = 7;  // Start filling bits from the MSB

  for (const unsigned char& encoded_byte : encoded) {
    uint8_t byte = encoded_byte;
    uint8_t bit_value = byte & 1;    // Extract the bit value (0 or 1)
    uint8_t run_length = byte >> 1;  // Extract the run length (7 bits)

    for (int i = 0; i < run_length; i++) {
      // Set the current bit in the byte
      current_byte |= (bit_value << bit_position);

      // Move to the next bit position
      if (bit_position == 0) {
        // Add the filled byte to decoded vector and reset for the next byte
        decoded.push_back(current_byte);
        current_byte = 0;
        bit_position = 7;
      } else {
        bit_position--;
      }
    }
  }

  // If there's an incomplete byte after the loop, add it to the decoded output
  if (bit_position < 7) {
    decoded.push_back(current_byte);
  }

  return decoded;
}
