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

#include "alib_compression.hpp"

// LZ77 encoding function with std::vector<unsigned char> as input
std::vector<unsigned char> alib_lz77_encode(const std::vector<unsigned char>& input,
                                            size_t window_size, size_t max_match_length) {
  std::vector<unsigned char> encoded;
  size_t input_size = input.size();
  size_t index = 0;

  while (index < input_size) {
    size_t best_offset = 0;
    size_t best_length = 0;
    unsigned char next_char = input[index];

    // Search in the sliding window for the longest match
    size_t search_start = (index >= window_size) ? index - window_size : 0;
    for (size_t j = search_start; j < index; ++j) {
      size_t match_length = 0;
      while (match_length < max_match_length && index + match_length < input_size &&
             input[j + match_length] == input[index + match_length]) {
        match_length++;
      }

      if (match_length > best_length) {
        best_length = match_length;
        best_offset = index - j;
        next_char = (index + match_length < input_size) ? input[index + match_length] : 0;
      }
    }

    // Store (offset, length, next_char) in `encoded` vector
    encoded.push_back(static_cast<unsigned char>((best_offset >> 8) & 0xFF));  // Offset MSB
    encoded.push_back(static_cast<unsigned char>(best_offset & 0xFF));         // Offset LSB
    encoded.push_back(static_cast<unsigned char>(best_length & 0xFF));         // Length
    encoded.push_back(next_char);                                              // Next character

    // Advance the index by the match length + 1 for the next character
    index += (best_length > 0) ? best_length + 1 : 1;
  }

  return encoded;
}

// LZ77 decoding function with std::vector<unsigned char>
std::vector<unsigned char> alib_lz77_decode(const std::vector<unsigned char>& encoded) {
  std::vector<unsigned char> decoded;
  size_t index = 0;

  while (index < encoded.size()) {
    // Read the offset (two bytes, big-endian)
    int offset = (static_cast<unsigned char>(encoded[index]) << 8) |
                 static_cast<unsigned char>(encoded[index + 1]);

    // Read the length (one byte)
    int length = static_cast<unsigned char>(encoded[index + 2]);

    // Read the next character
    unsigned char next_char = encoded[index + 3];

    // Append match from the decoded string if offset and length > 0
    size_t start_pos = decoded.size() - offset;
    for (int i = 0; i < length; ++i) {
      decoded.push_back(decoded[start_pos + i]);
    }

    // Append the next character
    decoded.push_back(next_char);

    // Move to the next (offset, length, next_char) triplet
    index += 4;
  }

  return decoded;
}
