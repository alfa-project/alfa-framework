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

#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <QObject>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include <vector>
#include <functional>
#include <memory>

//Timming
#include <chrono>
#include <time.h>
#include <QTimer>

#include <vector>

typedef std::map<std::string, std::vector<std::string>> topic_list_t;

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/kdtree/kdtree_flann.h>

//ALFA Includes
#include "alfa_msg/msg/config_message.hpp"
#include "alfa_msg/msg/alfa_metrics.hpp"
#include "alfa_msg/msg/alfa_alive_ping.hpp"
#include "alfa_msg/srv/alfa_configure.hpp"

class RosInterface : public QObject, public rclcpp::Node
{
    Q_OBJECT

public Q_SLOTS:
    void bag_timer_callback();

public:
    RosInterface(std::mutex* mutex);
    ~RosInterface();

    void connect_raw(QString new_topic);
    void connect_alfa(QString new_topic);

    //void publish_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr frame, std::string topic_name);
    void publish_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr frame, std::string topic_name, bool publish_in_monitor = true);
    void publish_pcl2_msg(const sensor_msgs::msg::PointCloud2& msg, std::string topic_name, bool publish_in_monitor = true);

    void load_bag(std::string file_name);
    void reload_bag();
    void publish_next_bag_pointcloud();
    void play_bag(float fps);
    void pause_bag();

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_frame;
    pcl::PointCloud<pcl::PointXYZI>::Ptr alfa_frame;

    bool* raw_playing_status;
    bool* alfa_playing_status;
    bool bag_loaded;

    QString raw_topic;
    QString alfa_topic;

    unsigned int raw_clouds;
    unsigned int alfa_clouds;

signals:
    void raw_cloud_received();
    void alfa_cloud_received();
    void metrics_received();
    void update_bag_status();
    void bag_ended();

private:
    std::string bag_file_name;
    std::string bag_topic;
    QTimer *bag_timer;

    std::mutex* mutex;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr alfa_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr monitor_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bag_publisher;
    
    void raw_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    void alfa_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

    rosbag2_cpp::readers::SequentialReader* bag_reader;

};


#endif // ROSINTERFACE_H