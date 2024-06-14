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

#ifndef METRICWINDOW_H
#define METRICWINDOW_H

#include "alfa_msg/msg/alfa_metrics.hpp"
#include "alfa_msg/msg/metric_message.hpp"
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rosinterface.h"

#include <QDialog>

using namespace std;

namespace Ui {
class MetricWindow;
}

class MetricWindow : public QDialog
{
    Q_OBJECT

public:
    explicit MetricWindow(string metric_topic, std::shared_ptr<RosInterface> ros_interface, QWidget *parent = nullptr);
    ~MetricWindow();

private slots:
    void on_lw_MetricList_currentRowChanged(int currentRow);
    void reject();
    void call_destructor();

signals:
    void destroy_object();

private:
    Ui::MetricWindow *ui;
    string metric_topic;
    std::shared_ptr<RosInterface> ros_interface;

    rclcpp::Subscription<alfa_msg::msg::AlfaMetrics>::SharedPtr metric_subscriber;
    //ros::Subscriber metric_subscriver;
    //ros::NodeHandle nh_metrics;


    void metrics_cb(const alfa_msg::msg::AlfaMetrics::SharedPtr metrics);
    alfa_msg::msg::AlfaMetrics::SharedPtr metrics_received;
    void update_index();
    uint current_index;

};

#endif // METRICWINDOW_H
