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

#ifndef CONFIGWINDOW_H
#define CONFIGWINDOW_H
#include "alfa_msg/srv/alfa_configure.hpp"
#include "alfa_msg/msg/config_message.hpp"
#include "alfa_msg/msg/alfa_alive_ping.hpp"
#include <vector>
#include <string>
//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "rosinterface.h"
#include <QDialog>

using namespace std;
namespace Ui {
class ConfigWindow;
}

class ConfigWindow : public QDialog
{
    Q_OBJECT

public:
    explicit ConfigWindow(string config_topic, std::shared_ptr<RosInterface> ros_interface, QWidget *parent = nullptr);
    explicit ConfigWindow(string config_topic,string config_tag, vector<alfa_msg::msg::ConfigMessage> defaults,QWidget *parent = nullptr);
    ~ConfigWindow();

private slots:
    void on_pb_SaveConfigs_clicked();

    void on_pb_LoadConfigs_clicked();

    void on_lw_ConfigsList_currentRowChanged(int currentRow);

    void on_pb_SubmitConfigs_clicked();

    void reject();
    void call_destructor();

signals:
    void destroy_object();

private:
    Ui::ConfigWindow *ui;
    string alive_topic;
    std::shared_ptr<RosInterface> ros_interface;
    
    rclcpp::Subscription<alfa_msg::msg::AlfaAlivePing>::SharedPtr ping_subscriber;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_subscriber;

    std::vector<alfa_msg::msg::ConfigMessage> configurations;
    //alfa_msg::srv::AlfaConfigure::Request configurations;

    //ros::ServiceClient configuration_client;
    int current_selected_parameter;
    bool set_node_parameters();
    void print_configs(alfa_msg::msg::ConfigMessage& configs);
    void load_configs_from_file(string filename);
    void publish_configs(alfa_msg::srv::AlfaConfigure  msg);
    void ping_callback(const alfa_msg::msg::AlfaAlivePing::SharedPtr message);

};

#endif // CONFIGWINDOW_H
