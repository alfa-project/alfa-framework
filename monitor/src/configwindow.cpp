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

#include "configwindow.h"

#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QToolTip>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "ui/ui_configwindow.h"

using std::placeholders::_1;

using namespace std;

ConfigWindow::ConfigWindow(string node_name, std::shared_ptr<RosInterface> ros_interface,
                           QWidget* parent)
    : QDialog(parent), ui(new Ui::ConfigWindow), ros_interface(ros_interface) {
  ui->setupUi(this);

  this->alive_topic = "/" + node_name + "_alive";
  ping_subscriber = ros_interface->create_subscription<alfa_msg::msg::AlfaAlivePing>(
      alive_topic, 10,  // Subscribe to the desired topic
      std::bind(&ConfigWindow::ping_callback, this, _1));

  client_subscriber = ros_interface->create_client<rcl_interfaces::srv::SetParameters>(
      "/" + node_name + "/set_parameters");

  ui->lb_topic_name->setText(QString::fromStdString(node_name));
  current_selected_parameter = 0;

  connect(this, SIGNAL(destroy_object()), this, SLOT(call_destructor()));
}

ConfigWindow::~ConfigWindow() {
  client_subscriber.reset();
  delete ui;
}

void ConfigWindow::on_pb_SaveConfigs_clicked() {
  QString filename = QFileDialog::getSaveFileName(this, tr("Save node configurations"), "",
                                                  tr("Config file (*.cfg)"));
  if (filename.isEmpty()) return;
  if (!filename.endsWith(".cfg")) filename.append(".cfg");

  ofstream MyFile(filename.toStdString());
  MyFile << "tag:" << ui->le_ConfigurationTag->text().toStdString() << endl;
  for (const auto& config : configurations) {
    MyFile << config.config_name << ":" << config.config << endl;
  }
  MyFile.close();
}

void ConfigWindow::on_pb_LoadConfigs_clicked() {
  QString filename =
      QFileDialog::getOpenFileName(this, tr("node configurations"), "", tr("Config file (*.cfg)"));
  if (filename.isEmpty()) return;
  load_configs_from_file(filename.toStdString());
}

void ConfigWindow::on_lw_ConfigsList_currentRowChanged(int currentRow) {
  if (currentRow >= 0 && currentRow < configurations.size()) {
    ui->config_name_label->setText(
        QString::fromStdString("Configuration name: " + configurations[currentRow].config_name));
    ui->sb_ConfigValue->setValue(configurations[currentRow].config);

    current_selected_parameter = currentRow;
  }
}

void ConfigWindow::on_pb_SubmitConfigs_clicked() {
  configurations[current_selected_parameter].config = ui->sb_ConfigValue->value();
  if (set_node_parameters())
    QMessageBox::information(this, "Result", "Configurations applied successfully");
  else
    QMessageBox::information(this, "Result", "Configurations failed, reason list on the console");
}

bool ConfigWindow::set_node_parameters() {
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  bool successful = true;

  for (const auto& config : configurations) {
    rcl_interfaces::msg::ParameterValue parameter;
    rcl_interfaces::msg::Parameter configuration;

    // Set parameter value
    parameter.type = 3;
    parameter.double_value = config.config;

    configuration.name = config.config_name;
    configuration.value = parameter;

    request->parameters.push_back(configuration);
  }

  while (!client_subscriber->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted, while waiting for the service. Exiting.");
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service");
  }

  auto return_message = client_subscriber->async_send_request(request);
  auto status = return_message.wait_until((std::chrono::system_clock::time_point)(
      std::chrono::system_clock::now() + std::chrono::seconds(5)));

  auto results = return_message.get()->results;
  for (const auto& result : results) {
    if (!result.successful) {
      std::cout << "Fail reason: " << result.reason << std::endl;
      successful = false;
    }
  }

  return successful;
}

void ConfigWindow::ping_callback(const alfa_msg::msg::AlfaAlivePing::SharedPtr message) {
  ui->lw_ConfigsList->clear();
  for (auto config : message->default_configurations) {
    QString to_print =
        QString::fromStdString(config.config_name) + ": " + QString::number(config.config);
    ui->lw_ConfigsList->addItem(to_print);

    std::vector<alfa_msg::msg::ConfigMessage>::iterator it =
        std::find(configurations.begin(), configurations.end(), config);
    if (it == configurations.end()) configurations.push_back(config);
  }
}

void ConfigWindow::load_configs_from_file(string filename) {
  fstream MyReadFile(filename);
  int index = 0;
  string line;
  configurations.clear();

  while (getline(MyReadFile, line)) {
    string text1, text2;

    size_t part1 = line.find(":");
    text1 = line.substr(0, part1);

    size_t part2 = line.find(":", part1 + 1);
    text2 = line.substr(part1 + 1, part2 - part1 - 1);

    if (index == 0) {
      ui->le_ConfigurationTag->setText(QString::fromStdString(text2));
    } else {
      alfa_msg::msg::ConfigMessage msg;
      msg.config_name = text1;
      stringstream s1;
      s1 << text2;
      s1 >> msg.config;
      configurations.push_back(msg);
    }
    index++;
  }

  set_node_parameters();
}

void ConfigWindow::reject() {
  QDialog::reject();
  emit destroy_object();
}

void ConfigWindow::call_destructor() { delete this; }