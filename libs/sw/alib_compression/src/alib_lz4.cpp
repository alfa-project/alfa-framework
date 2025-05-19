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

class BitWriter {
 public:
  std::vector<unsigned char> data;
  uint8_t current_byte = 0;
  uint8_t bit_position = 0;

  void write_bit(bool bit) {
    if (bit) {
      current_byte |= (1 << bit_position);
    }
    bit_position++;
    if (bit_position == 8) {
      data.push_back(current_byte);
      current_byte = 0;
      bit_position = 0;
    }
  }

  void write_bits(uint32_t value, size_t bit_count) {
    for (size_t i = 0; i < bit_count; i++) {
      write_bit((value >> i) & 1);
    }
  }

  void flush() {
    if (bit_position > 0) {
      data.push_back(current_byte);
    }
  }
};

// LZ4 encoding function
std::vector<unsigned char> alib_lz4_encode(const std::vector<unsigned char>& input,
                                           size_t min_match_length = 4, size_t max_offset = 65535,
                                           size_t max_literal_length = 15,
                                           size_t max_match_length = 19) {
  BitWriter writer;
  size_t input_size = input.size() * 8;  // Work with bits
  size_t input_index = 0;

  while (input_index < input_size) {
    size_t best_match_length = 0;
    size_t best_match_offset = 0;

    // Sliding window search for matches
    for (size_t j = (input_index > max_offset) ? input_index - max_offset : 0; j < input_index;
         j++) {
      size_t match_length = 0;

      while (match_length < max_match_length && input_index + match_length < input_size &&
             ((input[j / 8] >> (j % 8)) & 1) ==
                 ((input[(input_index + match_length) / 8] >> ((input_index + match_length) % 8)) &
                  1)) {
        match_length++;
      }

      if (match_length >= min_match_length && match_length > best_match_length) {
        best_match_length = match_length;
        best_match_offset = input_index - j;
      }
    }

    if (best_match_length >= min_match_length) {
      // Encode match
      writer.write_bits(best_match_length - min_match_length,
                        4);                      // Match length
      writer.write_bits(best_match_offset, 16);  // Offset

      input_index += best_match_length;
    } else {
      // Encode literals (no match found)
      size_t literal_start = input_index;
      size_t literal_length = 0;

      // Add literals until we find a match or hit the maximum literal length
      while (input_index < input_size &&
             literal_length < max_literal_length * 8) {  // Bit level length
        input_index++;
        literal_length++;
      }

      writer.write_bits(literal_length / 8,
                        4);  // Token with literal length in bytes

      // Add literal bits
      for (size_t i = literal_start; i < literal_start + literal_length; i++) {
        writer.write_bit((input[i / 8] >> (i % 8)) & 1);
      }
    }
  }

  writer.flush();
  return writer.data;
}

// LZ4 decoding function
std::vector<unsigned char> alib_lz4_decode(const std::vector<unsigned char>& encoded,
                                           size_t min_match_length = 4) {
  std::vector<unsigned char> decoded;
  size_t encoded_index = 0;
  size_t encoded_size = encoded.size();

  while (encoded_index < encoded_size) {
    uint8_t token = encoded[encoded_index++];

    // Decode literal length (low nibble of the token)
    size_t literal_length = token & 0x0F;

    // Copy literals to output
    if (literal_length > 0) {
      decoded.insert(decoded.end(), encoded.begin() + encoded_index,
                     encoded.begin() + encoded_index + literal_length);
      encoded_index += literal_length;
    }

    if (encoded_index >= encoded_size) break;  // End of data

    // Decode match offset (two bytes)
    uint16_t match_offset = static_cast<uint8_t>(encoded[encoded_index]) |
                            (static_cast<uint8_t>(encoded[encoded_index + 1]) << 8);
    encoded_index += 2;

    // Decode match length (high nibble of the token + minimum match length)
    size_t match_length = (token >> 4) + min_match_length;

    // Copy matched sequence from the output buffer
    for (size_t i = 0; i < match_length; i++) {
      decoded.push_back(decoded[decoded.size() - match_offset]);
    }
  }

  return decoded;
}
