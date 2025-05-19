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

#include "noisegenerator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ui/ui_noisegenerator.h"

NoiseGenerator::NoiseGenerator(vector<box> *boxlist, pcl::PointCloud<PointT>::Ptr inputCloud,
                               QWidget *parent)
    : QDialog(parent), ui(new Ui::NoiseGenerator) {
  ui->setupUi(this);
  this->boxlist = boxlist;
  this->inputCloud = inputCloud;
  pcl2_Header_seq = 0;
  ui->rbGaussian->setChecked(true);
}

void NoiseGenerator::add_noise() {
  for (auto mBox : *boxlist) {
    if (mBox.type == 2) {
      if (ui->rbGaussian->isChecked()) {
        pcl::PointXYZI center = calculate_center(mBox);
        do_gausian(mBox, center);
      } else if (ui->rbTrueRandom->isChecked()) {
        do_random(mBox);
      }
    }
  }
}

void NoiseGenerator::do_gausian(box target, pcl::PointXYZI center) {
  const double mean = ui->spinParam1->value();

  const double stddevx = (target.pEnd.x - center.x) / 2;
  const double stddevz = (target.pEnd.z - center.z) / 2;
  const double stddevy = (target.pEnd.y - center.y) / 2;

  std::mt19937 generator(std::random_device{}());

  std::default_random_engine generatorx;
  std::default_random_engine generatory;
  std::default_random_engine generatorz;

  std::normal_distribution<double> distx(mean, stddevx);
  std::normal_distribution<double> distz(mean, stddevz);
  std::normal_distribution<double> disty(mean, stddevy);

  for (int i = 0; i <= ui->spinNumberPoints->value(); i++) {
    pcl::PointXYZI noisePoint;
    noisePoint.x = center.x + distx(generator);
    noisePoint.y = center.y + disty(generator);
    noisePoint.z = center.z + distz(generator);

    inputCloud->push_back(noisePoint);
  }
}

void NoiseGenerator::do_random(box target) {
  float maxX = target.pEnd.x - (int)target.pStart.x;
  float maxY = target.pEnd.y - target.pStart.y;
  float maxZ = target.pEnd.z - target.pStart.z;
  srand(static_cast<unsigned>(time(0)));

  for (int i = 0; i <= ui->spinNumberPoints->value(); i++) {
    pcl::PointXYZI noisePoint;

    noisePoint.x =
        target.pStart.x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / maxX));
    noisePoint.y =
        target.pStart.y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / maxY));
    noisePoint.z =
        target.pStart.z + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / maxZ));
    inputCloud->push_back(noisePoint);
  }
}

pcl::PointXYZI NoiseGenerator::calculate_center(box target) {
  pcl::PointXYZI point;
  point.x = (target.pStart.x + target.pEnd.x) / 2.0;
  point.z = (target.pStart.z + target.pEnd.z) / 2.0;
  point.y = (target.pStart.y + target.pEnd.y) / 2.0;
  return point;
}

NoiseGenerator::~NoiseGenerator() { delete ui; }

void NoiseGenerator::on_pbAply_clicked() {
  add_noise();
  emit showPCloud();
}

void NoiseGenerator::on_rbGaussian_clicked() { ui->lbParam1->setText("Mean:"); }
