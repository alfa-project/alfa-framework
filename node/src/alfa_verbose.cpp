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

#include "alfa_node.hpp"

// Verbose functions
void AlfaNode::verbose_constr_chracteristics(string subscriber_topic, string node_name) {
  // Print the characteristics of the node
  std::cout << "--------------------------------------------------------" << std::endl;
  std::cout << "Starting ALFA node with the following settings:" << std::endl;
  std::cout << "Subscriber topic: " << subscriber_topic << std::endl;
  std::cout << "Name of the node: " << node_name << std::endl;
  std::cout << "Extension ID: " << configuration.extension_id << std::endl;
  std::cout << "Pointcloud ID: " << configuration.pointcloud_id << std::endl;
  std::cout << "Hardware Driver (SIU): " << configuration.hardware_support.hardware_driver
            << std::endl;
  std::cout << "Hardware Extension: " << configuration.hardware_support.hardware_extension
            << std::endl;
  std::cout << "--------------------------------------------------------" << std::endl;
}

void AlfaNode::verbose_begin(string function) {
  std::cout << "ALFA:" << function << "-> begin" << std::endl;
}

void AlfaNode::verbose_end(string function) {
  std::cout << "ALFA:" << function << "-> end" << std::endl;
}

void AlfaNode::verbose_ok(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << " OK" << std::endl;
}

void AlfaNode::verbose_fail(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << " FAIL" << std::endl;
}

void AlfaNode::verbose_info(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << std::endl;
}

void AlfaNode::verbose_not_defined(string function) {
  std::cout << "ALFA:" << function << "-> HARDWARE NOT DEFINED!!" << std::endl;
}
