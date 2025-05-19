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

#ifndef BOXSETTINGS_H
#define BOXSETTINGS_H

// Point Cloud Library
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QDialog>
#include <vector>

using namespace std;

struct box {
  pcl::PointXYZ pStart, pEnd;
  string name;
  unsigned int index;
  unsigned int type;  // 0-> Label Box 1->Noise Removal Box 2-> Noise Injection Box
};

namespace Ui {
class BoxSettings;
}

class BoxSettings : public QDialog {
  Q_OBJECT

 public:
  explicit BoxSettings(QWidget *parent = nullptr);
  explicit BoxSettings(QWidget *parent = nullptr, box *mBox = nullptr,
                       vector<box> *mListBox = nullptr);

  ~BoxSettings();

 private slots:
  void on_spinXStart_valueChanged(double arg1);
  void on_spinXEnd_valueChanged(double arg1);
  void on_spinYStart_valueChanged(double arg1);
  void on_spinYEnd_valueChanged(double arg1);
  void on_spinZStart_valueChanged(double arg1);
  void on_spinZEnd_valueChanged(double arg1);
  void on_spinMoveZ_valueChanged(double arg1);
  void on_spinMoveZ_editingFinished();
  void on_spinMoveX_valueChanged(double arg1);
  void on_spinMoveX_editingFinished();
  void on_spinMoveY_valueChanged(double arg1);
  void on_spinMoveY_editingFinished();
  void on_spinSensi_valueChanged(double arg1);
  void on_rbValue_clicked();
  void on_rbArrows_clicked();
  void on_cbLabelType_clicked();
  void on_cbNoiseRType_clicked();
  void on_cbNoiseInject_clicked();

 signals:
  void build_boxs();

 private:
  void updateSlider();
  Ui::BoxSettings *ui;
  vector<box> *boxlist;
  box *boxseletecd;
  box copy;
};

#endif  // BOXSETTINGS_H
