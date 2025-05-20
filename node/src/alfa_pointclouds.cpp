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

#include "alfa_node.hpp"

// Method for publishing a PointCloud2 type pointcloud
void AlfaNode::publish_pointcloud(sensor_msgs::msg::PointCloud2 &pointcloud) {
#ifdef ALFA_VERBOSE
  verbose_info("publish_pointcloud", "publishing pointcloud");
#endif
  pointcloud.header.frame_id =
      (string)this->get_name() +
      "_pointcloud";  // Create the pointcloud2 header to publish
  pointcloud.header.stamp = this->now();  // Get current time
  pointcloud_publisher->publish(
      pointcloud);  // Publish the point cloud in the ROS topic
}

// Method for publishing a PCL type pointcloud
void AlfaNode::publish_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud,
                                  std::uint32_t latency) {
  if (pointcloud == nullptr) pointcloud = this->output_pointcloud;
  this->configuration.latency = latency;
  sensor_msgs::msg::PointCloud2 pcl2_frame_temp;
  pcl::toROSMsg(
      *pointcloud,
      pcl2_frame_temp);  // convert the pcl object to the pointcloud2 one
  std::lock_guard<std::mutex> lk(pcl2_frame_condition_mutex);
  pcl2_frame_mutex.lock();
  pcl2_frame.push_back(pcl2_frame_temp);
  pcl2_frame_mutex.unlock();
  pcl2_frame_condition.notify_one();
}

pcl::PointCloud<AlfaPoint>::Ptr AlfaNode::get_input_pointcloud() {
#ifdef ALFA_VERBOSE
  verbose_info("get_input_pointcloud", "getting input pointcloud");
#endif
  return this->input_pointcloud;
}

pcl::PointCloud<AlfaPoint>::Ptr AlfaNode::get_output_pointcloud() {
#ifdef ALFA_VERBOSE
  verbose_info("get_output_pointcloud", "getting output pointcloud");
#endif
  return this->output_pointcloud;
}

std::vector<AlfaPoint> AlfaNode::get_input_pointcloud_as_vector() {
#ifdef ALFA_VERBOSE
  verbose_info("get_input_pointcloud", "getting input pointcloud");
#endif
  std::vector<AlfaPoint> r_input_pointcloud;

  for (const auto &point : *input_pointcloud)
    r_input_pointcloud.push_back(point);

  return r_input_pointcloud;
}

std::vector<AlfaPoint> AlfaNode::get_output_pointcloud_as_vector() {
#ifdef ALFA_VERBOSE
  verbose_info("get_output_pointcloud", "getting output pointcloud");
#endif
  std::vector<AlfaPoint> r_output_pointcloud;

  for (const auto &point : *output_pointcloud)
    r_output_pointcloud.push_back(point);

  return r_output_pointcloud;
}

// Returns the size of the input_pointcloud member
std::uint32_t AlfaNode::get_input_pointcloud_size() {
  return (std::uint32_t)input_pointcloud->size();
}

// Returns the size of the output_pointcloud member
std::uint32_t AlfaNode::get_output_pointcloud_size() {
  return (std::uint32_t)output_pointcloud->size();
}

// Returns true if the input_pointcloud is empty
bool AlfaNode::is_input_pointcloud_empty() {
  if (get_input_pointcloud_size() == 0)
    return true;
  else
    return false;
}

// Returns true if the output_pointcloud is empty
bool AlfaNode::is_output_pointcloud_empty() {
  if (get_output_pointcloud_size() == 0)
    return true;
  else
    return false;
}

// Returns true if the point_counter is pointing to the last point of the
// input_pointcloud, meaning the next call of
// get_point_sequentially_input_pointcloud(), will get the last point
bool AlfaNode::is_last_input_pointcloud_point() {
  if (is_input_pointcloud_empty() ||
      point_counter >= (get_input_pointcloud_size() - 1))
    return true;
  else
    return false;
}

void AlfaNode::push_point_output_pointcloud(AlfaPoint point) {
  output_mutex.lock();
  output_pointcloud->push_back(point);
  output_mutex.unlock();
}

// Get specific point from input cloud, using the point argument. Returns true
// if successfull, false otherwise
bool AlfaNode::get_point_input_pointcloud(std::uint32_t position,
                                          AlfaPoint &point) {
  if (position < get_input_pointcloud_size() - 1) {
    point = (*input_pointcloud)[position];
    return true;
  }

  return false;
}

// Get specific point from input cloud, using the point index. Returns the point
// if successfull, the last point otherwise
AlfaPoint AlfaNode::get_point_input_pointcloud(std::uint32_t position) {
  if (position <= get_input_pointcloud_size() - 1) {
    return (*input_pointcloud)[position];
  } else
    return (*input_pointcloud)[get_input_pointcloud_size() - 1];
}

// Get specific point from output cloud, using the point index. Returns the
// point if successfull, the last point otherwise
AlfaPoint AlfaNode::get_point_output_pointcloud(std::uint32_t position) {
  if (position <= get_output_pointcloud_size() - 1) {
    return (*output_pointcloud)[position];
  } else
    return (*output_pointcloud)[get_input_pointcloud_size() - 1];
}

// Multiple calls, give points from the input_pointcloud squentially, return
// true if there are points left, false otherwise
bool AlfaNode::get_point_input_pointcloud(AlfaPoint &point) {
  if (point_counter < get_input_pointcloud_size()) {
    input_mutex.lock();
    point = (*input_pointcloud)[point_counter++];
    input_mutex.unlock();

    return true;
  }

  return false;
}

bool AlfaNode::reset_input_pointcloud_counter() {
  if (is_input_pointcloud_empty()) return false;

  point_counter = 0;
  return true;
}

// Set specific custom field value in the output_pointcloud position, returns
// true if successfull, false otherwise
bool AlfaNode::set_custom_field_output_pointcloud(std::uint32_t position,
                                                  std::uint32_t value) {
  if (position < (get_output_pointcloud_size() - 1)) {
    (*output_pointcloud)[position].custom_field = value;
    return true;
  }

  return false;
}

#pragma GCC optimize("prefetch-loop-arrays")
void AlfaNode::convert_msg_to_pointcloud() {
  // Lock and retrieve the front element of the deque
  ros_pointcloud_mutex.lock();
  auto ros_pointcloud_temp = ros_pointcloud.front();
  ros_pointcloud.pop_front();
  ros_pointcloud_mutex.unlock();

  const auto &data = ros_pointcloud_temp.data;
  const auto &fields = ros_pointcloud_temp.fields;

  // Find the field in the message to convert into custom field
  int field_offset = -1;  // Initialize to an invalid value
  int field_offset_2 = -1;

  int field_type = 0;
  int field_type_2 = 0;

  auto custom_type = this->configuration.custom_field_conversion_type;

  // Common loop for finding the field offsets based on field names
  auto findFieldOffsetByName = [&](const std::string &fieldName = "",
                                   const std::string &fieldName2 = "") {
    for (const auto &field : fields) {
      if (fieldName == "")
        break;
      else if (field.name == fieldName) {
        field_offset = field.offset;
        field_type = field.datatype;
        break;
      }
    }

    for (const auto &field : fields) {
      if (fieldName2 == "")
        break;
      else if (field.name == fieldName2) {
        field_offset_2 = field.offset;
        field_type_2 = field.datatype;
        break;
      }
    }
  };

  switch (custom_type) {
    case CUSTOM_FIELD_USER:
      findFieldOffsetByName("USER");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is USER");
#endif
      break;

    case CUSTOM_FIELD_INTENSITY:
      findFieldOffsetByName("", "intensity");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is intensity");
#endif
      break;

    case CUSTOM_FIELD_LABEL:
      findFieldOffsetByName("", "label");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is label");
#endif
      break;

    case CUSTOM_FIELD_RGB:
      findFieldOffsetByName("", "rgb");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is rgb");
#endif
      break;

    case CUSTOM_FIELD_FILTER:
      findFieldOffsetByName("", "intensity");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is INTENSITY");
#endif
      break;

    case CUSTOM_FIELD_INTENSITY_LABEL:
      findFieldOffsetByName("intensity", "label");
#ifdef ALFA_VERBOSE
      verbose_info(
          "main_thread",
          "convert_msg_to_pointcloud: field name is INTENSITY and LABEL");
#endif
      break;

    default:
      findFieldOffsetByName("intensity");
#ifdef ALFA_VERBOSE
      verbose_info(
          "main_thread",
          "convert_msg_to_pointcloud: field name is default: INTENSITY");
#endif
      break;
  }

#ifdef ALFA_VERBOSE
  verbose_info("main_thread",
               "convert_msg_to_pointcloud: offset calculation ok");
#endif
  // Clear the existing data in input_pointcloud and reserve memory
  input_pointcloud->clear();
  input_pointcloud->reserve(ros_pointcloud_temp.width *
                            ros_pointcloud_temp.height);

#ifdef ALFA_VERBOSE
  verbose_info("main_thread", "convert_msg_to_pointcloud: converting points");
  cout << "data.size(): " << data.size() << endl;
  cout << "ros_pointcloud_temp.point_step: " << ros_pointcloud_temp.point_step
       << endl;
#endif

  // Iterate through the data and populate custom point cloud
  for (size_t i = 0; i < data.size(); i += ros_pointcloud_temp.point_step) {
    AlfaPoint point;
    // Extract x, y, z from the data (assuming these fields are at offsets
    // 0, 4, and 8 respectively)
    memcpy(&(point.x), &data[i + fields[0].offset], sizeof(float));
    memcpy(&(point.y), &data[i + fields[1].offset], sizeof(float));
    memcpy(&(point.z), &data[i + fields[2].offset], sizeof(float));

    point.custom_field = 0;

    // Extract the custom field from the data
    auto fillCustomField = [&](const std::uint32_t &field_offset,
                               const std::uint32_t &field_type,
                               const bool &byte_selector) {
      switch (field_type) {
        case sensor_msgs::msg::PointField::FLOAT32: {  // Intensity
          float temp_float;
          memcpy(&temp_float, &data[i + field_offset], sizeof(float));
          uint8_t temp_8;
          // Handle intensity values in 0-1 range and 0-100 range
          if (temp_float <= 1) {
            temp_8 = static_cast<uint8_t>(temp_float * 100);
          } else
            temp_8 = static_cast<uint8_t>(temp_float);
          if (byte_selector)
            point.custom_field |= temp_8 << 8;
          else
            point.custom_field |= temp_8;
          break;
        }

        case sensor_msgs::msg::PointField::UINT16: {  // Label
          std::uint16_t temp_uint16;
          memcpy(&temp_uint16, &data[i + field_offset], sizeof(std::uint16_t));
          uint8_t temp_8 = 0;
          try {
            temp_8 = semantic_kitti_to_alfa.at(static_cast<int>(temp_uint16));

          } catch (const std::out_of_range &e) {
            temp_8 = 0;
          }
          if (byte_selector)
            point.custom_field |= temp_8 << 8;
          else
            point.custom_field |= temp_8;
          break;
        }

        default:
          point.custom_field = 0;
          break;
      }
    };

    // Fill the custom field
    if (field_offset != -1) fillCustomField(field_offset, field_type, true);
    if (field_offset_2 != -1)
      fillCustomField(field_offset_2, field_type_2, false);

    input_pointcloud->emplace_back(std::move(point));  // Use emplace_back
  }

#ifdef ALFA_VERBOSE
  verbose_info("main_thread", "convert_msg_to_pointcloud: done");
#endif
}

// ALFA to SemanticKITTI mapping
const std::unordered_map<int, int> alfa_to_semantic_kitti = {
    {ALFA_LABEL_UNLABELED, SEMANTIC_UNLABELED},
    {ALFA_LABEL_OUTLIER, SEMANTIC_OUTLIER},
    {ALFA_LABEL_CAR, SEMANTIC_CAR},
    {ALFA_LABEL_BICYCLE, SEMANTIC_BICYCLE},
    {ALFA_LABEL_BUS, SEMANTIC_BUS},
    {ALFA_LABEL_MOTORCYCLE, SEMANTIC_MOTORCYCLE},
    {ALFA_LABEL_ON_RAILS, SEMANTIC_ON_RAILS},
    {ALFA_LABEL_TRUCK, SEMANTIC_TRUCK},
    {ALFA_LABEL_OTHER_VEHICLE, SEMANTIC_OTHER_VEHICLE},
    {ALFA_LABEL_PERSON, SEMANTIC_PERSON},
    {ALFA_LABEL_BICYCLIST, SEMANTIC_BICYCLIST},
    {ALFA_LABEL_MOTORCYCLIST, SEMANTIC_MOTORCYCLIST},
    {ALFA_LABEL_ROAD, SEMANTIC_ROAD},
    {ALFA_LABEL_PARKING, SEMANTIC_PARKING},
    {ALFA_LABEL_SIDEWALK, SEMANTIC_SIDEWALK},
    {ALFA_LABEL_OTHER_GROUND, SEMANTIC_OTHER_GROUND},
    {ALFA_LABEL_BUILDING, SEMANTIC_BUILDING},
    {ALFA_LABEL_FENCE, SEMANTIC_FENCE},
    {ALFA_LABEL_OTHER_STRUCTURE, SEMANTIC_OTHER_STRUCTURE},
    {ALFA_LABEL_LANE_MARKING, SEMANTIC_LANE_MARKING},
    {ALFA_LABEL_VEGETATION, SEMANTIC_VEGETATION},
    {ALFA_LABEL_TRUNK, SEMANTIC_TRUNK},
    {ALFA_LABEL_TERRAIN, SEMANTIC_TERRAIN},
    {ALFA_LABEL_POLE, SEMANTIC_POLE},
    {ALFA_LABEL_TRAFFIC_SIGN, SEMANTIC_TRAFFIC_SIGN},
    {ALFA_LABEL_OTHER_OBJECT, SEMANTIC_OTHER_OBJECT},
    {ALFA_LABEL_MOVING_CAR, SEMANTIC_MOVING_CAR},
    {ALFA_LABEL_MOVING_BICYCLIST, SEMANTIC_MOVING_BICYCLIST},
    {ALFA_LABEL_MOVING_PERSON, SEMANTIC_MOVING_PERSON},
    {ALFA_LABEL_MOVING_MOTORCYCLIST, SEMANTIC_MOVING_MOTORCYCLIST},
    {ALFA_LABEL_MOVING_ON_RAILS, SEMANTIC_MOVING_ON_RAILS},
    {ALFA_LABEL_MOVING_BUS, SEMANTIC_MOVING_BUS},
    {ALFA_LABEL_MOVING_TRUCK, SEMANTIC_MOVING_TRUCK},
    {ALFA_LABEL_MOVING_OTHER_VEHICLE, SEMANTIC_MOVING_OTHER_VEHICLE}};

// SemanticKITTI to ALFA mapping
const std::unordered_map<int, int> semantic_kitti_to_alfa = {
    {SEMANTIC_UNLABELED, ALFA_LABEL_UNLABELED},
    {SEMANTIC_OUTLIER, ALFA_LABEL_OUTLIER},
    {SEMANTIC_CAR, ALFA_LABEL_CAR},
    {SEMANTIC_BICYCLE, ALFA_LABEL_BICYCLE},
    {SEMANTIC_BUS, ALFA_LABEL_BUS},
    {SEMANTIC_MOTORCYCLE, ALFA_LABEL_MOTORCYCLE},
    {SEMANTIC_ON_RAILS, ALFA_LABEL_ON_RAILS},
    {SEMANTIC_TRUCK, ALFA_LABEL_TRUCK},
    {SEMANTIC_OTHER_VEHICLE, ALFA_LABEL_OTHER_VEHICLE},
    {SEMANTIC_PERSON, ALFA_LABEL_PERSON},
    {SEMANTIC_BICYCLIST, ALFA_LABEL_BICYCLIST},
    {SEMANTIC_MOTORCYCLIST, ALFA_LABEL_MOTORCYCLIST},
    {SEMANTIC_ROAD, ALFA_LABEL_ROAD},
    {SEMANTIC_PARKING, ALFA_LABEL_PARKING},
    {SEMANTIC_SIDEWALK, ALFA_LABEL_SIDEWALK},
    {SEMANTIC_OTHER_GROUND, ALFA_LABEL_OTHER_GROUND},
    {SEMANTIC_BUILDING, ALFA_LABEL_BUILDING},
    {SEMANTIC_FENCE, ALFA_LABEL_FENCE},
    {SEMANTIC_OTHER_STRUCTURE, ALFA_LABEL_OTHER_STRUCTURE},
    {SEMANTIC_LANE_MARKING, ALFA_LABEL_LANE_MARKING},
    {SEMANTIC_VEGETATION, ALFA_LABEL_VEGETATION},
    {SEMANTIC_TRUNK, ALFA_LABEL_TRUNK},
    {SEMANTIC_TERRAIN, ALFA_LABEL_TERRAIN},
    {SEMANTIC_POLE, ALFA_LABEL_POLE},
    {SEMANTIC_TRAFFIC_SIGN, ALFA_LABEL_TRAFFIC_SIGN},
    {SEMANTIC_OTHER_OBJECT, ALFA_LABEL_OTHER_OBJECT},
    {SEMANTIC_MOVING_CAR, ALFA_LABEL_MOVING_CAR},
    {SEMANTIC_MOVING_BICYCLIST, ALFA_LABEL_MOVING_BICYCLIST},
    {SEMANTIC_MOVING_PERSON, ALFA_LABEL_MOVING_PERSON},
    {SEMANTIC_MOVING_MOTORCYCLIST, ALFA_LABEL_MOVING_MOTORCYCLIST},
    {SEMANTIC_MOVING_ON_RAILS, ALFA_LABEL_MOVING_ON_RAILS},
    {SEMANTIC_MOVING_BUS, ALFA_LABEL_MOVING_BUS},
    {SEMANTIC_MOVING_TRUCK, ALFA_LABEL_MOVING_TRUCK},
    {SEMANTIC_MOVING_OTHER_VEHICLE, ALFA_LABEL_MOVING_OTHER_VEHICLE}};
