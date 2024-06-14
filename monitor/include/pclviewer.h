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

#pragma once

// Qt
#include <QMainWindow>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <QWidgetItem>
#include <QListWidgetItem>
#include <boost/math/special_functions/round.hpp>
#include "boxsettings.h"
#include <vtkRenderWindow.h>
#include "noisegenerator.h"
#include <thread>
#include <mutex>
#include "rosinterface.h"
//#include "alfanode.h"

#define TIMER_INTERVAL 1000 //reloading time in ms

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer(int argc, char **argv, QWidget *parent = 0);

  ~PCLViewer();

public Q_SLOTS:
  void saveScreenButtonPressed();

  void loadFileButtonPressed();
  void resetCameraPressed();
  void axisChosen();

  void lookUpTableChosen();
  static void mouseEventClick(const pcl::visualization::PointPickingEvent &event, void *viewer_void);
  void timer_callback();

protected:
  pcl::visualization::PCLVisualizer::Ptr viewer_input;
  pcl::visualization::PCLVisualizer::Ptr viewer_output;

  PointCloudT::Ptr raw_cloud;

  int filtering_axis_;

  int color_mode_;

  void colorCloudDistances(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int=nullptr);

private slots:
  void on_btnAddBox_clicked();
  void on_listWidget_itemClicked(QListWidgetItem *item);
  void on_listWidget_itemDoubleClicked(QListWidgetItem *item);
  void on_btnDeletedSelected_clicked();
  void buidbox_slot();

  //void print_nodes();

  void on_btnSaveBox_clicked();
  void on_btnLoadBoxs_clicked();

  void on_listWidget_itemEntered(QListWidgetItem *item);
  void show_alfa_cloud();
  void update_bag_status();
  void on_pbDeletePoints_clicked();
  void on_pbSaveFile_clicked();
  void update_pointcloud(bool is_display_frame = false);
  void on_pbAddNoise_clicked();

  void on_cbShow_Alfa_clicked();

  void on_save_camera_clicked();

  void on_load_camera_clicked();

  void on_sync_cameras_clicked();



  void on_pb_PointParametersHint_pressed();

  //void on_pb_reload1_clicked();

  void on_cb_RAW_activated(const QString &arg1);

  void on_cb_ALFA_activated(const QString &arg1);

  void on_pbRawPlay_clicked();

  void on_pbAlfaPlay_clicked();

  void on_pb_select_bag_clicked();

  void on_pb_NextFrame_clicked();

  void on_sb_FPS_valueChanged(double arg1);

  void on_pb_PlayBag_clicked();

  void on_rb_Loop_clicked();

  void on_rb_Output_clicked();

  void on_cb_RemovePoints_clicked();

  void on_cb_InjectNoise_clicked();


  void on_l_ExecOrder_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

  //void check_for_rosmaster();

  void on_lw_confignodes_itemDoubleClicked(QListWidgetItem *item);

  void on_lw_metrics_itemDoubleClicked(QListWidgetItem *item);

  void on_cbHide_clicked();

  void on_le_TopicName_textEdited(const QString &arg1);

  void on_pb_SendFrame_clicked();

private:
  void saveBoxToFile();
  bool btnAddBox;
  bool showFiltered;
  void buildbox();
  void buildbox(int index);
  void calculate_insidePoints();
  void calculate_insidePoints(int index);
  bool insidepoint(box mBoxl, pcl::PointXYZI p1);
  void filter_decision(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void do_deletePointBox(pcl::PointCloud<pcl::PointXYZI>::Ptr output);
  void to_color_pcloud(PointCloudT::Ptr input,bool is_alfa_cloud=0);
  bool update_node_status();

  void t_calback_thread();
  std::thread* timer_response;

  vector<box> boxlist;
  vector<string> configurable_nodes;

  Ui::PCLViewer *ui;
  PointCloudT::Ptr alfa_cloud;
  int argc;
  char **argv;
  QString rosnode;
  bool deletePointBox;
  bool addNoiseBox;
  std::thread *m_ros_thread;
  void run(bool is_display_frame = false);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  std::mutex mutex;
  NoiseGenerator *noise;
  bool enMultithreading;
  bool outputToTopic;
  void set_style();

  bool load_topics();
  std::shared_ptr<RosInterface> ros_interface;
  bool raw_playing_status;
  bool alfa_playing_status;
  QTimer *timer;
  void read_loading_cloud();
  void read_alfa_cloud();
  void t_ros_master();

  void ros_spinner();

  bool ros_master_owner;

  std::thread* bag_thread;
  std::thread* master_thread;
  std::thread* spinner;

  uint alfa_pd_filter_selected;
  topic_list_t topics;
  vector<string> old_services;
  vector<string> pcl_topics;
  vector<string> metric_topics;
  vector<string> alfa_nodes;
  //vector<AlfaNode*> nodes;
  uint8_t exec_order[2];
  int metric_index_ui;
};
