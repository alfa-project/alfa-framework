
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

#ifndef ALIB_COMPRESSION_H
#define ALIB_COMPRESSION_H

#include <iostream>
#include <unordered_map>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"

// Generic function
void write_binary_file(const std::string& file_path, const std::vector<unsigned char>& data);
void print_frequencies(const std::vector<unsigned char>& input);
void print_vector_binary(const std::vector<unsigned char>& input);
void output_compression_code(const std::vector<unsigned char>& encoded_data);
void output_decoded_compression_code(const std::vector<unsigned char>& encoded_data);

std::vector<AlfaPoint> convert_code_to_AlfaPoint_vector(const std::vector<unsigned char>& code);
std::vector<unsigned char> convert_AlfaPoint_vector_to_code(
    const std::vector<AlfaPoint>& pointcloud);

// RLE
std::vector<unsigned char> alib_rle_encode(const std::vector<unsigned char>& input);
std::vector<unsigned char> alib_rle_decode(const std::vector<unsigned char>& encoded);

// LZ4
std::vector<unsigned char> alib_lz4_encode(const std::vector<unsigned char>& input,
                                           size_t min_match_length, size_t max_offset,
                                           size_t max_literal_length, size_t max_match_length);
std::vector<unsigned char> alib_lz4_decode(const std::vector<unsigned char>& encoded,
                                           size_t min_match_length);

// LZ77
std::vector<unsigned char> alib_lz77_encode(const std::vector<unsigned char>& input,
                                            size_t window_size, size_t max_match_length);
std::vector<unsigned char> alib_lz77_decode(const std::vector<unsigned char>& encoded);

// Huffman
std::vector<unsigned char> alib_huffman_encode(const std::vector<unsigned char>& input);
std::vector<unsigned char> alib_huffman_decode(const std::vector<unsigned char>& encoded);

// Simplified Huffman
std::vector<unsigned char> alib_huffman_s_encode(const std::vector<unsigned char>& input);
std::vector<unsigned char> alib_huffman_s_decode(const std::vector<unsigned char>& encoded);

#endif  // ALIB_COMPRESSION_H