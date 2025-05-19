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

#include "rosinterface.h"

#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

using std::placeholders::_1;

// Global QoS variable
rclcpp::QoS qos(rclcpp::KeepLast(10));

RosInterface::RosInterface(std::mutex* mutex) : Node("alfa_monitor"), mutex(mutex) {
  monitor_publisher = nullptr;
  bag_publisher = nullptr;
  raw_topic = "";
  alfa_topic = "";
  bag_loaded = false;
  bag_reader = new rosbag2_cpp::readers::SequentialReader();

  bag_timer = new QTimer(this);
  connect(bag_timer, SIGNAL(timeout()), this, SLOT(bag_timer_callback()));

  // Configure global QoS settings
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
}

RosInterface::~RosInterface() { bag_reader->close(); }

void RosInterface::connect_raw(QString new_topic) {
  if (new_topic.compare("Disconnected")) {
    raw_subscriber.reset();  // Clean any subscription

    raw_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        new_topic.toStdString(), qos,  // Subscribe with global QoS
        std::bind(&RosInterface::raw_callback, this, _1));

    raw_topic = new_topic;
  } else {
    raw_subscriber.reset();  // Clean any subscription
    raw_topic = "Disconnected";
  }
}

void RosInterface::connect_alfa(QString new_topic) {
  if (new_topic.compare("Disconnected")) {
    alfa_subscriber.reset();  // Clean any subscription

    alfa_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        new_topic.toStdString(), qos,  // Subscribe with global QoS
        std::bind(&RosInterface::alfa_callback, this, _1));

    alfa_topic = new_topic;
  } else {
    alfa_subscriber.reset();  // Clean any subscription
    raw_topic = "Disconnected";
  }
}

void RosInterface::raw_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  static unsigned int index = 0;
  try {
    if ((cloud->width * cloud->height) == 0) return;

    if (*raw_playing_status) {
      if (cloud->height * cloud->width == 0) return;

      mutex->lock();
      pcl::fromROSMsg(*cloud, *raw_frame);
      raw_clouds++;
      mutex->unlock();
      index++;
      emit raw_cloud_received();
    }
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
  }
}

void RosInterface::alfa_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  try {
    if ((cloud->width * cloud->height) == 0) return;

    if (*alfa_playing_status) {
      if (cloud->height * cloud->width == 0) return;

      mutex->lock();
      pcl::fromROSMsg(*cloud, *alfa_frame);
      alfa_clouds++;
      mutex->unlock();
      emit alfa_cloud_received();
    }
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
  }
}

void RosInterface::publish_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr frame,
                                       std::string topic_name, bool publish_in_monitor) {
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  if (publish_in_monitor)
    publisher = monitor_publisher;
  else
    publisher = bag_publisher;

  if (publisher == nullptr || topic_name != publisher->get_topic_name()) {
    publisher.reset();
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, qos);
  }

  // Message preparation and publishing
  sensor_msgs::msg::PointCloud2 pcl2_frame;
  pcl::toROSMsg(*frame, pcl2_frame);

  pcl2_frame.header.frame_id =
      "alfa_monitor_pointcloud";          // Create the pointcloud2 header to publish
  pcl2_frame.header.stamp = this->now();  // Get current time

  publisher->publish(pcl2_frame);  // Publish the point cloud in the ROS topic
}

void RosInterface::publish_pcl2_msg(const sensor_msgs::msg::PointCloud2& msg,
                                    std::string topic_name, bool publish_in_monitor) {
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  if (publish_in_monitor)
    publisher = monitor_publisher;
  else
    publisher = bag_publisher;

  if (publisher == nullptr || topic_name != publisher->get_topic_name()) {
    publisher.reset();
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, qos);
  }

  publisher->publish(msg);  // Publish the point cloud in the ROS topic
}

void RosInterface::load_bag(std::string file_name) {
  bag_file_name = file_name;  // Update file name
  // Define loading options
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = file_name;
  storage_options.storage_id = "sqlite3";

  // Setup converter options to read the bag
  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // Load the bag
  if (!bag_loaded) {
    bag_reader->open(storage_options, converter_options);  // Load the bag
    bag_loaded = true;
  } else {
    bag_reader->close();
    bag_reader->open(storage_options, converter_options);  // Load the bag
  }

  // Find topics with pointcloud2 topics and store them for publishing
  const auto topics = bag_reader->get_all_topics_and_types();
  for (auto const& topic : topics) {
    if (QString::fromStdString(topic.type).contains("PointCloud2")) bag_topic = topic.name;
  }

  // Publish first frame
  publish_next_bag_pointcloud();
}

void RosInterface::reload_bag() { load_bag(bag_file_name); }

void RosInterface::publish_next_bag_pointcloud() {
  sensor_msgs::msg::PointCloud2 msg;
  auto type_support =
      rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>();

  if (bag_loaded) {
    while (bag_reader->has_next()) {
      auto serialized_message = bag_reader->read_next();
      rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
      auto current_topic = serialized_message->topic_name;

      if (bag_topic == current_topic)  // It is the topic with pointcloud2 data
      {
        auto deserializer = rclcpp::SerializationBase(type_support);
        deserializer.deserialize_message(&extracted_serialized_msg, &msg);
        publish_pcl2_msg(msg, bag_topic, false);
        return;
      }
    }

    bag_reader->close();
    bag_loaded = false;
    emit bag_ended();
  }
}

void RosInterface::bag_timer_callback() { publish_next_bag_pointcloud(); }

void RosInterface::play_bag(float fps) { bag_timer->start((1 / fps) * 1000); }

void RosInterface::pause_bag() { bag_timer->stop(); }
