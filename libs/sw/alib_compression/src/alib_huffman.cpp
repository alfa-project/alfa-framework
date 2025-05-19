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

#include <bitset>
#include <iostream>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Node structure for Huffman Tree
struct HuffmanNode {
  unsigned char data;
  int freq;
  std::shared_ptr<HuffmanNode> left, right;

  HuffmanNode(unsigned char data, int freq)
      : data(data), freq(freq), left(nullptr), right(nullptr) {}

  bool operator>(const HuffmanNode &other) const { return freq > other.freq; }
};

// Function to build Huffman Tree
std::shared_ptr<HuffmanNode> buildHuffmanTree(
    const std::unordered_map<unsigned char, int> &freq_map) {
  auto comp = [](std::shared_ptr<HuffmanNode> left, std::shared_ptr<HuffmanNode> right) {
    return left->freq > right->freq;
  };
  std::priority_queue<std::shared_ptr<HuffmanNode>, std::vector<std::shared_ptr<HuffmanNode>>,
                      decltype(comp)>
      pq(comp);

  for (const auto &pair : freq_map) {
    pq.push(std::make_shared<HuffmanNode>(pair.first, pair.second));
  }

  while (pq.size() > 1) {
    auto left = pq.top();
    pq.pop();
    auto right = pq.top();
    pq.pop();
    auto parent = std::make_shared<HuffmanNode>(0, left->freq + right->freq);
    parent->left = left;
    parent->right = right;
    pq.push(parent);
  }

  return pq.top();
}

// Function to generate Huffman Codes from the tree
void generateHuffmanCodes(const std::shared_ptr<HuffmanNode> &node, const std::string &code,
                          std::unordered_map<unsigned char, std::string> &huffmanCode) {
  if (!node) return;

  if (!node->left && !node->right) {
    huffmanCode[node->data] = code;
  }

  generateHuffmanCodes(node->left, code + "0", huffmanCode);
  generateHuffmanCodes(node->right, code + "1", huffmanCode);
}

// Huffman encoding function
std::vector<unsigned char> alib_huffman_encode(const std::vector<unsigned char> &input) {
  // Calculate frequency of each character
  std::unordered_map<unsigned char, int> freq_map;
  for (unsigned char ch : input) {
    freq_map[ch]++;
  }

  // Build Huffman Tree
  std::shared_ptr<HuffmanNode> root = buildHuffmanTree(freq_map);

  // Generate Huffman Codes
  std::unordered_map<unsigned char, std::string> huffmanCode;
  generateHuffmanCodes(root, "", huffmanCode);

  // Encode frequency map into header
  std::vector<unsigned char> encoded;
  encoded.push_back(
      static_cast<unsigned char>(freq_map.size()));  // Store number of unique characters
  for (const auto &pair : freq_map) {
    encoded.push_back(pair.first);                               // Store character
    encoded.push_back(static_cast<unsigned char>(pair.second));  // Store frequency
  }

  // Encode the input data
  std::stringstream encoded_stream;
  for (const unsigned char &ch : input) {
    encoded_stream << huffmanCode[ch];
  }

  std::string encoded_str = encoded_stream.str();
  for (size_t i = 0; i < encoded_str.size(); i += 8) {
    std::string byte_str = encoded_str.substr(i, 8);
    if (byte_str.size() < 8) {
      byte_str.append(8 - byte_str.size(), '0');
    }
    unsigned char byte = static_cast<unsigned char>(std::bitset<8>(byte_str).to_ulong());
    encoded.push_back(byte);
  }

  return encoded;
}

// Huffman decoding function
std::vector<unsigned char> alib_huffman_decode(const std::vector<unsigned char> &encoded) {
  // Parse the frequency map from the encoded data
  size_t index = 0;
  int num_unique_chars = encoded[index++];
  std::unordered_map<unsigned char, int> freq_map;

  for (int i = 0; i < num_unique_chars; ++i) {
    unsigned char ch = encoded[index++];
    int freq = encoded[index++];
    freq_map[ch] = freq;
  }

  // Rebuild Huffman Tree
  std::shared_ptr<HuffmanNode> root = buildHuffmanTree(freq_map);

  // Decode the data
  std::vector<unsigned char> decoded;
  std::shared_ptr<HuffmanNode> current = root;
  for (size_t i = index; i < encoded.size(); ++i) {
    std::bitset<8> bits(encoded[i]);
    for (int j = 7; j >= 0; --j) {
      current = bits[j] ? current->right : current->left;
      if (!current->left && !current->right) {
        decoded.push_back(current->data);
        current = root;
      }
    }
  }

  return decoded;
}
