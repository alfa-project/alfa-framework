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

#include "metricwindow.h"

#include <QMessageBox>
#include <iostream>
#include <thread>

#include "ui/ui_metricwindow.h"

using std::placeholders::_1;

MetricWindow::MetricWindow(string metric_topic, std::shared_ptr<RosInterface> ros_interface,
                           QWidget* parent)
    : QDialog(parent), ui(new Ui::MetricWindow), ros_interface(ros_interface) {
  ui->setupUi(this);
  this->metric_topic = metric_topic;

  metric_subscriber = ros_interface->create_subscription<alfa_msg::msg::AlfaMetrics>(
      metric_topic, 10,  // Subscribe to the desired topic
      std::bind(&MetricWindow::metrics_cb, this, _1));

  connect(this, SIGNAL(destroy_object()), this, SLOT(call_destructor()));

  current_index = 0;
}

MetricWindow::~MetricWindow() {
  metric_subscriber.reset();
  delete ui;
}

void MetricWindow::reject() {
  QDialog::reject();
  emit destroy_object();
}

void MetricWindow::call_destructor() { delete this; }

void MetricWindow::on_lw_MetricList_currentRowChanged(int currentRow) {
  if (currentRow >= 0 && currentRow < metrics_received->metrics.size()) {
    current_index = currentRow;
    update_index();
  }
}

void MetricWindow::metrics_cb(const alfa_msg::msg::AlfaMetrics::SharedPtr metrics) {
  try {
    metrics_received = metrics;
    ui->lb_MetricTag->setText(QString::fromStdString(metrics->message_tag) + "");
    ui->lw_MetricList->clear();

    for (auto metric : metrics->metrics) {
      QString to_print = QString::fromStdString(metric.metric_name) + ": " +
                         QString::number(metric.metric, 'f', 1) + " " +
                         QString::fromStdString(metric.units);
      ui->lw_MetricList->addItem(to_print);
    }

    update_index();
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
  }
}

void MetricWindow::update_index() {
  if (current_index >= 0 && current_index < metrics_received->metrics.size()) {
    ui->lb_MetricName->setText(
        QString::fromStdString(metrics_received->metrics[current_index].metric_name));
    ui->lb_MetricValue->setNum(metrics_received->metrics[current_index].metric);
    ui->lb_MetricUnits->setText(
        QString::fromStdString(metrics_received->metrics[current_index].units));
  }
}
