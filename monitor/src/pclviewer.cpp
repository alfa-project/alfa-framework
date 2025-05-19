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

#include "pclviewer.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/common/common.h>

#include <QDebug>
#include <QMessageBox>

#include "ui/ui_pclviewer.h"
//#include "ros/ros.h"
#include <QFile>
#include <QMetaType>
#include <QTimer>
#include <QToolTip>
#include <stdexcept>

#include "clouds.h"
#include "configwindow.h"
#include "math.h"
#include "metricwindow.h"
#include "style_sheat.h"
#include "vtkGenericOpenGLRenderWindow.h"

Ui::PCLViewer *AuxUi;
vector<box> *AuxBoxList;
using namespace std;
pcl::visualization::PCLVisualizer::Ptr *AuxViewer;
bool *AuxAddBox;
pcl::PointCloud<pcl::PointXYZI>::Ptr auxCloud;

using namespace pcl;

PCLViewer::PCLViewer(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::PCLViewer),
      filtering_axis_(1),  // = y
      color_mode_(4)       // = Rainbow
{
  ui->setupUi(this);
  this->setWindowTitle("ALFA-Monitor");
  ui->w_alfa->hide();
  set_style();
  AuxUi = ui;
  AuxViewer = &viewer_input;
  AuxBoxList = &boxlist;
  AuxAddBox = &btnAddBox;
  btnAddBox = false;
  rosnode = "null";
  raw_cloud.reset(new PointCloudT);
  alfa_cloud.reset(new PointCloudT);
  raw_cloud->resize(500);
  this->argc = argc;
  this->argv = argv;
  deletePointBox = false;
  auxCloud = raw_cloud;

  // Setup point cloud rendering
  auto renderer_input = vtkSmartPointer<vtkRenderer>::New();
  auto renderer_output = vtkSmartPointer<vtkRenderer>::New();

  auto render_input_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  auto render_output_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();

  render_input_window->AddRenderer(renderer_input);
  render_output_window->AddRenderer(renderer_output);

  viewer_input.reset(
      new pcl::visualization::PCLVisualizer(renderer_input, render_input_window, "viewer", false));
  viewer_output.reset(new pcl::visualization::PCLVisualizer(renderer_output, render_output_window,
                                                            "viewer", false));
  viewer_input->setBackgroundColor(0.13, 0.13, 0.13);
  viewer_output->setBackgroundColor(0.13, 0.13, 0.13);

  ui->qvtkWidget->setRenderWindow(viewer_input->getRenderWindow());
  ui->qvtkWidget_alfa->setRenderWindow(viewer_output->getRenderWindow());

  viewer_input->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
  viewer_output->setupInteractor(ui->qvtkWidget_alfa->interactor(),
                                 ui->qvtkWidget_alfa->renderWindow());

  ui->qvtkWidget->renderWindow()->Render();
  ui->qvtkWidget_alfa->renderWindow()->Render();

  to_color_pcloud(raw_cloud);
  to_color_pcloud(raw_cloud, 1);

  viewer_input->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                 "cloud");
  viewer_output->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                                  "cloud");
  viewer_input->registerPointPickingCallback(mouseEventClick, (void *)&viewer_input);
  viewer_input->resetCamera();
  viewer_input->addOrientationMarkerWidgetAxes(ui->qvtkWidget->interactor());
  std::vector<pcl::visualization::Camera> cameras;
  viewer_input->getCameras(cameras);
  viewer_output->setCameraParameters(cameras[0]);

  ui->qvtkWidget->renderWindow()->Render();
  ui->qvtkWidget_alfa->renderWindow()->Render();
  ;

  // Connect all the buttons' signals to their functions
  connect(ui->pushButton_load, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
  connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(saveScreenButtonPressed()));
  connect(ui->pushReset, SIGNAL(clicked()), this, SLOT(resetCameraPressed()));

  connect(ui->radioButton_x, SIGNAL(clicked()), this, SLOT(axisChosen()));
  connect(ui->radioButton_y, SIGNAL(clicked()), this, SLOT(axisChosen()));
  connect(ui->radioButton_z, SIGNAL(clicked()), this, SLOT(axisChosen()));

  connect(ui->radioButton_BlueRed, SIGNAL(clicked()), this, SLOT(lookUpTableChosen()));
  connect(ui->radioButton_Rainbow, SIGNAL(clicked()), this, SLOT(lookUpTableChosen()));

  enMultithreading = false;
  outputToTopic = false;

  noise = new NoiseGenerator(&boxlist, raw_cloud, this);

  ros_interface = std::make_shared<RosInterface>(&mutex);
  ros_interface->raw_frame = raw_cloud;
  ros_interface->alfa_frame = alfa_cloud;

  spinner = new std::thread(&PCLViewer::ros_spinner, this);  // Spin node

  // connect(this, SIGNAL(refresh_raw_cloud()),this,SLOT(update_pointcloud()));
  // connect(this, SIGNAL(refresh_alfa_cloud()),this,SLOT(show_alfa_cloud()));

  connect(ros_interface.get(), SIGNAL(bag_ended()), this, SLOT(update_bag_status()));
  connect(ros_interface.get(), SIGNAL(raw_cloud_received()), this, SLOT(update_pointcloud()));
  connect(ros_interface.get(), SIGNAL(alfa_cloud_received()), this, SLOT(show_alfa_cloud()));

  ui->cb_RAW->addItem("Disconnected");
  ui->cb_ALFA->addItem("Disconnected");
  read_alfa_cloud();
  alfa_playing_status = true;
  raw_playing_status = true;
  ros_interface->alfa_playing_status = &alfa_playing_status;
  ros_interface->raw_playing_status = &raw_playing_status;

  on_cbShow_Alfa_clicked();

  // timer settings
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
  timer->start(TIMER_INTERVAL);

  alfa_pd_filter_selected = 0;

  exec_order[0] = 0;
  exec_order[1] = 0;
  on_rb_Output_clicked();
}

PCLViewer::~PCLViewer() { delete ui; }

void PCLViewer::loadFileButtonPressed() {
  QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home",
                                                  tr("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
  PointCloudT::Ptr cloud_tmp(new PointCloudT);

  if (filename.isEmpty()) return;

  int return_status;
  if (filename.endsWith(".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

  if (return_status != 0) {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud(*cloud_tmp, *raw_cloud);
  else {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud(*cloud_tmp, *raw_cloud, vec);
  }
  to_color_pcloud(raw_cloud);
  viewer_input->resetCamera();
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::mouseEventClick(const pcl::visualization::PointPickingEvent &event,
                                void *viewer_inputvoid) {
  if (event.getPointIndex() == -1) return;
  pcl::PointXYZ p1, p2;
  int pIndex = event.getPointIndex();

  event.getPoint(p1.x, p1.y, p1.z);
  stringstream pX, pY, pZ, pI;
  pX << setprecision(3) << p1.x;
  pY << setprecision(3) << p1.y;
  pZ << setprecision(3) << p1.z;
  pI << setprecision(3) << (*auxCloud)[pIndex].intensity;

  AuxUi->lb_pX->setText(QString::fromStdString(pX.str()));
  AuxUi->lb_pY->setText(QString::fromStdString(pY.str()));
  AuxUi->lb_pZ->setText(QString::fromStdString(pZ.str()));
  AuxUi->lb_pI->setText(QString::fromStdString(pI.str()));

  if (*AuxAddBox) {
    p2.x = p1.x + AuxUi->spinSize->value();
    p2.y = p1.y + AuxUi->spinSize->value();
    p2.z = p1.z + AuxUi->spinSize->value();
    box newBox;
    newBox.pStart = p1;
    newBox.pEnd = p2;
    newBox.index = AuxBoxList->size();
    newBox.name = "Cube" + to_string(newBox.index);
    newBox.type = 0;
    *AuxAddBox = false;
    (*AuxViewer)->addCube(p1.x, p2.x, p1.y, p2.y, p1.z, p2.z, 1.0, 1.0, 1.0, newBox.name, 0);
    AuxUi->btnAddBox->setText("Add Box");
    AuxBoxList->push_back(newBox);
    AuxUi->listWidget->addItem(QString::fromStdString(newBox.name));
  }
}

void PCLViewer::timer_callback() {
  timer_response = new std::thread(&PCLViewer::t_calback_thread, this);
}

void PCLViewer::saveScreenButtonPressed() {
  QString filename =
      QFileDialog::getSaveFileName(this, tr("Image Data"), "/home", tr("PNG (*.png *.jpeg)"));

  if (filename.isEmpty()) filename = "Screenshot";
  if (filename.endsWith(".pcd", Qt::CaseInsensitive))
    filename.remove(".pcd");
  else if (filename.endsWith(".ply", Qt::CaseInsensitive))
    filename.remove(".ply");
  if (ui->cbShow_Alfa->isChecked()) {
    QString filename_alfa = filename;
    filename_alfa.append("_alfa.png");
    viewer_output->saveScreenshot(filename_alfa.toStdString());
  }
  filename.append(".png");
  viewer_input->saveScreenshot(filename.toStdString());
}

void PCLViewer::axisChosen() {
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio
  // buttons
  if (ui->radioButton_x->isChecked())
    filtering_axis_ = 0;

  else if (ui->radioButton_y->isChecked())
    filtering_axis_ = 1;

  else
    filtering_axis_ = 2;

  to_color_pcloud(raw_cloud);
  to_color_pcloud(alfa_cloud, true);
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::lookUpTableChosen() {
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio
  // buttons
  if (ui->radioButton_BlueRed->isChecked())
    color_mode_ = 0;

  else
    color_mode_ = 4;

  to_color_pcloud(raw_cloud);
  to_color_pcloud(alfa_cloud, true);
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::resetCameraPressed() {
  viewer_input->resetCameraViewpoint("cloud");
  viewer_input->resetCamera();
  viewer_output->resetCameraViewpoint("cloud");
  viewer_output->resetCamera();
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::colorCloudDistances(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int) {
  // Find the minimum and maximum values along the selected axis
  double min, max, max_len;
  // Set an initial value

  switch (filtering_axis_) {
    case 0:  // x
      min = (*cloud)[0].x;
      max = (*cloud)[0].x;
      break;
    case 1:  // y
      min = (*cloud)[0].y;
      max = (*cloud)[0].y;
      break;
    default:  // z
      min = (*cloud)[0].z;
      max = (*cloud)[0].z;
      break;
  }
  max_len = sqrt(pow((*cloud)[0].x, 2) + pow((*cloud)[0].y, 2) + pow((*cloud)[0].z, 2));

  // Search for the minimum/maximum
  if (filtering_axis_ != 0)
    for (PointCloud<PointXYZRGBA>::iterator cloud_it = cloud->begin(); cloud_it != cloud->end();
         ++cloud_it) {
      switch (filtering_axis_) {
        case 1:  // y
          if (min > cloud_it->y) min = cloud_it->y;

          if (max < cloud_it->y) max = cloud_it->y;
          break;
        default:  // z
          if (min > cloud_it->z) min = cloud_it->z;

          if (max < cloud_it->z) max = cloud_it->z;
          if (max_len < sqrt(pow(cloud_it->x, 2) + pow(cloud_it->y, 2) + pow(cloud_it->z, 2))) {
            max_len = sqrt(pow(cloud_it->x, 2) + pow(cloud_it->y, 2) + pow(cloud_it->z, 2));
          }
          break;
      }
    }
  else
    for (PointCloud<PointXYZI>::iterator cloud_it = cloud_int->begin();
         cloud_it != cloud_int->end(); ++cloud_it) {
      if (min > cloud_it->intensity) min = cloud_it->intensity;

      if (max < cloud_it->intensity) max = cloud_it->intensity;
      if (max_len < cloud_it->intensity) {
        max_len = cloud_it->intensity;
      }
    }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)     // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost
  int pointer = 0;
  for (PointCloud<PointXYZRGBA>::iterator cloud_it = cloud->begin(); cloud_it != cloud->end();
       ++cloud_it) {
    int value;
    switch (filtering_axis_) {
      case 0:  // x
        value = std::lround(((*cloud_int)[pointer].intensity * -min) *
                            lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = std::lround((cloud_it->y - min) * lut_scale);
        break;
      default:  // Distance
        value = std::lround(
            sqrt((pow(cloud_it->x, 2) + pow(cloud_it->y, 2) + pow(cloud_it->z + 10, 2))) * 300 /
            (max_len / 1.1));
        break;
    }
    if (value > 255) value = 255;

    // Apply color to the cloud
    switch (color_mode_) {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128) {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        } else {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128
                          ? 2 * value
                          : 255 - ((value - 128) * 2);      // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
                                                            // b[0] = 255, b[128] = 0
    }
    pointer++;
  }
}

void PCLViewer::on_btnAddBox_clicked() {
  if (btnAddBox) {
    btnAddBox = false;
    ui->btnAddBox->setText("Add Box");
  } else {
    btnAddBox = true;
    ui->btnAddBox->setText("Shift+Click");
  }
}

void PCLViewer::buildbox() {
  viewer_input->removeAllShapes();
  calculate_insidePoints();
  if (!ui->cbHide->isChecked()) {
    for (int i = 0; i < boxlist.size(); i++) {
      viewer_input->addCube(boxlist[i].pStart.x, boxlist[i].pEnd.x, boxlist[i].pStart.y,
                            boxlist[i].pEnd.y, boxlist[i].pStart.z, boxlist[i].pEnd.z, 1.0, 1.0,
                            1.0, boxlist[i].name, 0);
    }
    ui->qvtkWidget->renderWindow()->Render();
  }
}

void PCLViewer::buildbox(int index) {
  viewer_input->removeAllShapes();
  calculate_insidePoints(index);
  if (!ui->cbHide->isChecked()) {
    for (int i = 0; i < boxlist.size(); i++) {
      if (index != i)
        viewer_input->addCube(boxlist[i].pStart.x, boxlist[i].pEnd.x, boxlist[i].pStart.y,
                              boxlist[i].pEnd.y, boxlist[i].pStart.z, boxlist[i].pEnd.z, 1.0, 1.0,
                              1.0, boxlist[i].name, 0);
      else
        viewer_input->addCube(boxlist[i].pStart.x, boxlist[i].pEnd.x, boxlist[i].pStart.y,
                              boxlist[i].pEnd.y, boxlist[i].pStart.z, boxlist[i].pEnd.z, 1.0, 0, 0,
                              boxlist[i].name, 0);
    }
    ui->qvtkWidget->renderWindow()->Render();
  }
}

void PCLViewer::calculate_insidePoints() {
  int count = 0;
  for (auto mbox : boxlist)
    for (auto &point : *raw_cloud) {
      if (insidepoint(mbox, point)) count++;
    }
  ui->lbTotal->setText(QString::fromStdString(to_string(count)));
}

void PCLViewer::calculate_insidePoints(int index) {
  calculate_insidePoints();
  int count = 0;
  for (auto &point : *raw_cloud) {
    if (insidepoint(boxlist[index], point)) count++;
  }
  ui->lbInside->setText(QString::fromStdString(to_string(count)));
}

bool PCLViewer::insidepoint(box mBoxl, PointXYZI p1) {
  if (p1.x >= mBoxl.pStart.x && p1.x <= mBoxl.pEnd.x) {
    if (p1.y >= mBoxl.pStart.y && p1.y <= mBoxl.pEnd.y) {
      if (p1.z >= mBoxl.pStart.z && p1.z <= mBoxl.pEnd.z) return true;
    }
  }
  return false;
}

void PCLViewer::on_listWidget_itemClicked(QListWidgetItem *item) {
  int row = ui->listWidget->currentRow();
  buildbox(row);
}

void PCLViewer::on_listWidget_itemDoubleClicked(QListWidgetItem *item) {
  BoxSettings *newBoxSet = new BoxSettings(this, &boxlist[ui->listWidget->currentRow()], &boxlist);

  connect(newBoxSet, SIGNAL(build_boxs()), this, SLOT(buidbox_slot()));
  newBoxSet->show();
}

void PCLViewer::on_btnDeletedSelected_clicked() {
  int row = ui->listWidget->currentRow();
  boxlist.erase(boxlist.begin() + row);
  QListWidgetItem *item = ui->listWidget->currentItem();
  delete ui->listWidget->takeItem(ui->listWidget->row(item));
  buildbox();
}

void PCLViewer::buidbox_slot() {
  int index = ui->listWidget->currentRow();

  calculate_insidePoints(index);

  if (!ui->cbHide->isChecked()) {
    viewer_input->removeAllShapes();

    for (int i = 0; i < boxlist.size(); i++) {
      if (index != i)
        viewer_input->addCube(boxlist[i].pStart.x, boxlist[i].pEnd.x, boxlist[i].pStart.y,
                              boxlist[i].pEnd.y, boxlist[i].pStart.z, boxlist[i].pEnd.z, 1.0, 1.0,
                              1.0, boxlist[i].name, 0);
      else
        viewer_input->addCube(boxlist[i].pStart.x, boxlist[i].pEnd.x, boxlist[i].pStart.y,
                              boxlist[i].pEnd.y, boxlist[i].pStart.z, boxlist[i].pEnd.z, 1.0, 0, 0,
                              boxlist[i].name, 0);
    }
    ui->qvtkWidget->renderWindow()->Render();
  }
}

void PCLViewer::on_btnSaveBox_clicked() { saveBoxToFile(); }

void PCLViewer::saveBoxToFile() {
  QString fileName = QFileDialog::getSaveFileName(this, tr("Box File"), "/home/",
                                                  tr("BoxFile (*.bxf);;All Files (*)"));
  if (fileName.isEmpty()) return;
  fileName.append(".bxf");
  ofstream MyFile(fileName.toStdString());
  for (int i = 0; i < boxlist.size(); i++) {
    MyFile << boxlist[i].pStart.x << ";" << boxlist[i].pEnd.x << ";" << boxlist[i].pStart.y << ";"
           << boxlist[i].pEnd.y << ";" << boxlist[i].pStart.z << ";" << boxlist[i].pEnd.z << ";"
           << boxlist[i].index << ";" << boxlist[i].name << ";" << boxlist[i].type << "\n";
  }
}

void PCLViewer::on_btnLoadBoxs_clicked() {
  string myText = "";
  QString filename = QFileDialog::getOpenFileName(this, tr("Box File"), "/home/",
                                                  tr("BoxFile (*.bxf);;All Files (*)"));
  fstream MyReadFile(filename.toStdString());
  while (getline(MyReadFile, myText)) {
    string sx, ex, sy, ey, sz, ez, index, name, type;
    box newBox;
    std::size_t foundsx = myText.find(";");
    sx = myText.substr(0, foundsx);
    size_t foundex = myText.find(";", foundsx + 1);
    ex = myText.substr(foundsx + 1, foundex - foundsx - 1);
    size_t foundsy = myText.find(";", foundex + 1);
    sy = myText.substr(foundex + 1, foundsy - foundex - 1);
    size_t foundey = myText.find(";", foundsy + 1);
    ey = myText.substr(foundsy + 1, foundey - foundsy - 1);

    size_t foundsz = myText.find(";", foundey + 1);
    sz = myText.substr(foundey + 1, foundsz - foundey - 1);

    size_t foundez = myText.find(";", foundsz + 1);
    ez = myText.substr(foundsz + 1, foundez - foundsz - 1);

    size_t foundindex = myText.find(";", foundez + 1);
    index = myText.substr(foundez + 1, foundindex - foundez - 1);
    size_t foundName = myText.find(";", foundindex + 1);
    name = myText.substr(foundindex + 1, foundName - foundindex - 1);

    type = myText.substr(foundName + 1);

    stringstream ssx, sex, ssy, sey, ssz, sez, stype;

    float fsx, fex, fsy, fey, fsz, fez;
    int ftype;
    ssx << sx;
    ssx >> fsx;
    sex << ex;
    sex >> fex;
    ssy << sy;
    ssy >> fsy;
    sey << ey;
    sey >> fey;
    ssz << sz;
    ssz >> fsz;
    sez << ez;
    sez >> fez;
    stype << type;
    stype >> ftype;
    newBox.pStart.x = fsx;
    newBox.pEnd.x = fex;
    newBox.pStart.y = fsy;
    newBox.pEnd.y = fey;
    newBox.pStart.z = fsz;
    newBox.pEnd.z = fez;
    newBox.index = stoi(index);
    newBox.name = name;
    newBox.type = ftype;
    boxlist.push_back(newBox);
    ui->listWidget->addItem(QString::fromStdString(newBox.name));
  }
  buildbox();
}

void PCLViewer::on_listWidget_itemEntered(QListWidgetItem *item) { buildbox(); }

void PCLViewer::show_alfa_cloud() {
  to_color_pcloud(alfa_cloud, 1);
  ui->lb_alfa_points->setText(QString::number(alfa_cloud->size()));
}

void PCLViewer::update_bag_status() {
  if (!ui->rb_Loop->isChecked()) {
    if (!ui->pb_PlayBag->text().contains("Restart"))
      ui->pb_PlayBag->setText("Restart");  // Need to update interface

    else
      ui->pb_PlayBag->setText("Play");
  } else
    ros_interface->reload_bag();  // If loop is checked and we reached here, we have to reload
}

void PCLViewer::on_pbDeletePoints_clicked() {
  do_deletePointBox(raw_cloud);
  to_color_pcloud(raw_cloud);
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::on_pbSaveFile_clicked() {
  QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "/home",
                                                  tr("Point cloud data (*.pcd *.ply)"));

  if (filename.isEmpty()) return;

  int return_status;

  if (filename.endsWith(".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileASCII(filename.toStdString(), *raw_cloud);
  else if (filename.endsWith(".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *raw_cloud);
  else {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *raw_cloud);
  }

  if (return_status != 0) {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
    return;
  }
}

void PCLViewer::update_pointcloud(bool is_display_frame) {
  run(is_display_frame);
  ui->lb_raw_points->setText(QString::number(raw_cloud->size()));
}

void PCLViewer::filter_decision(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  to_color_pcloud(raw_cloud);
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::do_deletePointBox(pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < boxlist.size(); i++) {
    if (boxlist[i].type == 1) {
      pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "z", pcl::ComparisonOps::GT, boxlist[i].pStart.z)));
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "z", pcl::ComparisonOps::LT, boxlist[i].pEnd.z)));
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "x", pcl::ComparisonOps::GT, boxlist[i].pStart.x)));
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "x", pcl::ComparisonOps::LT, boxlist[i].pEnd.x)));
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "y", pcl::ComparisonOps::GT, boxlist[i].pStart.y)));
      range_cond->addComparison(
          pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(
              "y", pcl::ComparisonOps::LT, boxlist[i].pEnd.y)));
      pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
      condrem.setCondition(range_cond);
      condrem.setInputCloud(raw_cloud);
      condrem.filter(*filtered);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      int index = 0;
      bool to_remove;

      for (auto &point : *raw_cloud) {
        to_remove = false;
        for (auto &point2 : *filtered) {
          if (point.x == point2.x)
            if (point.y == point2.y)
              if (point.z == point2.z) {
                to_remove = true;
                break;
              }
        }
        if (!to_remove) {
          inliers->indices.push_back(index);
        }
        index++;
      }
      extract.setInputCloud(raw_cloud);
      extract.setIndices(inliers);
      extract.filter(*output);
    }
  }
}

void PCLViewer::to_color_pcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input, bool is_alfa_cloud) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_pcloud;
  color_pcloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (input->width * input->height == 0)  // If point cloud empty
    return;

  mutex.lock();
  pcl::copyPointCloud(*input, *color_pcloud);
  mutex.unlock();

  colorCloudDistances(color_pcloud, input);
  if (is_alfa_cloud) {
    viewer_output->removeAllPointClouds();
    viewer_output->addPointCloud(color_pcloud, "cloud");
    ui->qvtkWidget_alfa->renderWindow()->Render();

  } else {
    viewer_input->removeAllPointClouds();
    viewer_input->addPointCloud(color_pcloud, "cloud");
    ui->qvtkWidget->renderWindow()->Render();
  }
}

bool PCLViewer::update_node_status() {
  bool needs_update = false;
  /*for(auto &node :nodes)
  {
      if(node->update_status())
          needs_update = true;
  }*/
  return needs_update;
}

void PCLViewer::t_calback_thread() {
  load_topics();
  try {
    mutex.lock();
    double raw_fps =
        (ros_interface->raw_clouds / (TIMER_INTERVAL / 1000.0)) * (1000 / TIMER_INTERVAL);
    double alfa_fps =
        (ros_interface->alfa_clouds / (TIMER_INTERVAL / 1000.0)) * (1000 / TIMER_INTERVAL);

    ros_interface->raw_clouds = 0;
    ros_interface->alfa_clouds = 0;

    ui->lb_raw_fps->setText(QString::number(raw_fps));
    ui->lb_alfa_fps->setText(QString::number(alfa_fps));
    mutex.unlock();

  } catch (const std::exception &e) {
    cout << e.what() << endl;
  }
}

void PCLViewer::run(bool is_display_frame) {
  if (ui->rb_Output->isChecked()) {
    for (int i = 0; i < 2; i++) {
      switch (exec_order[i]) {
        case 1:
          do_deletePointBox(raw_cloud);
          break;

        case 2:
          noise->add_noise();
          break;

        default:
          break;
      }
    }
    if (!is_display_frame) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr msg_cloud;
      msg_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::copyPointCloud(*raw_cloud, *msg_cloud);
      ros_interface->publish_point_cloud(msg_cloud, ui->le_TopicName->text().toStdString());
    }
  }
  to_color_pcloud(ros_interface->raw_frame);
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::set_style() { this->setStyleSheet(combinear); }

bool PCLViewer::load_topics() {
  int index = 0, raw_current_index = 0, alfa_current_index = 0;

  topic_list_t current_topics = ros_interface->get_topic_names_and_types();

  pcl_topics.clear();
  metric_topics.clear();
  alfa_nodes.clear();

  for (auto const &[name, type] : current_topics) {
    if (QString::fromStdString(type[0]).contains("PointCloud2")) {
      index++;
      pcl_topics.push_back(name);

      if (name == ros_interface->raw_topic.toStdString()) raw_current_index = index;

      if (name == ros_interface->alfa_topic.toStdString()) alfa_current_index = index;

    } else if (QString::fromStdString(type[0]).contains("AlfaMetrics")) {
      metric_topics.push_back(name);
    } else if (QString::fromStdString(type[0]).contains("AlfaAlivePing")) {
      // Store node's name since it is an active alfa node
      std::string temp = name;
      temp.erase(0, 1);

      size_t found = temp.find_last_of('_');
      temp.erase(found);

      alfa_nodes.push_back(temp);
    }
  }

  // Update PCL UI
  // Clear Interface
  ui->cb_RAW->clear();
  ui->cb_ALFA->clear();

  ui->cb_RAW->addItem("Disconnected");
  ui->cb_ALFA->addItem("Disconnected");

  for (const auto &topic : pcl_topics) {
    ui->cb_RAW->addItem(QString::fromStdString(
        topic));  // + "--------->" + QString::fromStdString(std::string(payload[i][1])));
    ui->cb_ALFA->addItem(QString::fromStdString(topic));
  }
  ui->cb_RAW->setCurrentIndex(raw_current_index);
  ui->cb_ALFA->setCurrentIndex(alfa_current_index);

  // Metrics UI
  ui->lw_metrics->clear();
  for (const auto &topic : metric_topics) {
    ui->lw_metrics->addItem(QString::fromStdString(topic));
  }

  // Config UI
  ui->lw_confignodes->clear();
  for (const auto &name : alfa_nodes) {
    ui->lw_confignodes->addItem(QString::fromStdString(name));
  }

  return false;
}

void PCLViewer::read_loading_cloud() {
  string x, y, z, i;
  int parameter_index = 0;
  pcl::PointCloud<pcl::PointXYZI> new_cloud;
  for (auto m_char : loading) {
    if (m_char == ' ') {
      parameter_index++;
    } else if (m_char == ';') {
      parameter_index = 0;
      // cout<<"point read:X"<<x<<" Y:"<<y<<" Z:"<<z<<endl;
      stringstream sx, sy, sz;
      float fx, fy, fz;
      sx << x;
      sx >> fx;

      sy << y;
      sy >> fy;

      sz << z;
      sz >> fz;
      pcl::PointXYZI point;
      point.x = fx;
      point.y = fy;
      point.z = fz;
      new_cloud.push_back(point);

      x = "";
      y = "";
      z = "";
      i = "";
    } else {
      switch (parameter_index) {
        case 0:
          x += m_char;
          break;
        case 1:
          y += m_char;
          break;
        case 2:
          z += m_char;
          break;
        case 3:
          i += m_char;
      }
    }
  }

  *raw_cloud = new_cloud;
  // update_pointcloud(true);
  viewer_input->resetCamera();
  // ui->qvtkWidget->renderWindow()->Render();
  resetCameraPressed();
}

void PCLViewer::read_alfa_cloud() {
  string x, y, z, i;
  int parameter_index = 0;
  pcl::PointCloud<pcl::PointXYZI> new_cloud;
  for (auto m_char : alfa) {
    if (m_char == ' ') {
      parameter_index++;
    } else if (m_char == ';') {
      parameter_index = 0;
      stringstream sx, sy, sz;
      float fx, fy, fz;
      sx << x;
      sx >> fx;

      sy << y;
      sy >> fy;

      sz << z;
      sz >> fz;
      pcl::PointXYZI point;
      point.x = fx;
      point.y = fy;
      point.z = fz;
      new_cloud.push_back(point);

      x = "";
      y = "";
      z = "";
      i = "";
    } else {
      switch (parameter_index) {
        case 0:
          x += m_char;
          break;
        case 1:
          y += m_char;
          break;
        case 2:
          z += m_char;
          break;
        case 3:
          i += m_char;
      }
    }
  }

  *raw_cloud = new_cloud;
  *alfa_cloud = new_cloud;
  update_pointcloud(true);
  show_alfa_cloud();
  viewer_input->resetCamera();
  viewer_output->resetCamera();
  ui->qvtkWidget->renderWindow()->Render();
}

void PCLViewer::on_pbAddNoise_clicked() {
  // connect(noise, SIGNAL(showPCloud()), this, SLOT(update_pointcloud()));
  noise->show();
}

void PCLViewer::on_cbShow_Alfa_clicked() {
  if (ui->cbShow_Alfa->isChecked()) {
    ui->w_alfa->show();
    ui->w_alfa_pc_run->show();
    ui->w_points_alfa->show();
    ui->w_fps_alfa->show();
  } else {
    ui->w_alfa_pc_run->hide();
    ui->w_alfa->hide();
    ui->w_points_alfa->hide();
    ui->w_fps_alfa->hide();
  }
}

void PCLViewer::on_save_camera_clicked() {
  std::vector<pcl::visualization::Camera> cameras;

  viewer_input->getCameras(cameras);
  viewer_output->setCameraParameters(cameras[0]);
  QString fileName = QFileDialog::getSaveFileName(this, tr("Cam File"), "~/",
                                                  tr("cam file (*.cam);;All Files (*)"));
  viewer_input->saveCameraParameters(fileName.toStdString() + ".cam");
}

void PCLViewer::on_load_camera_clicked() {
  std::vector<pcl::visualization::Camera> cameras;

  viewer_input->getCameras(cameras);
  viewer_output->setCameraParameters(cameras[0]);
  QString filename = QFileDialog::getOpenFileName(this, tr("Cam File"), "~/",
                                                  tr("CamFile (*.cam);;All Files (*)"));
  viewer_input->loadCameraParameters(filename.toStdString());
  viewer_input->getCameras(cameras);
  viewer_output->setCameraParameters(cameras[0]);
}

void PCLViewer::on_sync_cameras_clicked() {
  std::vector<pcl::visualization::Camera> cameras;
  viewer_input->getCameras(cameras);
  viewer_output->setCameraParameters(cameras[cameras.size() - 1]);
  // viewer_output->updateCamera();
}

void PCLViewer::on_pb_PointParametersHint_pressed() {
  QToolTip::showText(ui->pb_PointParametersHint->mapToGlobal(QPoint()),
                     "Shift+Left Click to obtain a point's parameters");
}

void PCLViewer::on_cb_RAW_activated(const QString &arg1) {
  ros_interface->connect_raw(arg1);
  if (raw_playing_status) {
    ui->pbRawPlay->setText("Pause");
    return;

  } else {
    ui->pbRawPlay->setText("Play");
    return;
  }
}

void PCLViewer::on_cb_ALFA_activated(const QString &arg1) {
  ros_interface->connect_alfa(arg1);
  if (alfa_playing_status) {
    ui->pbAlfaPlay->setText("Pause");
    return;

  } else {
    ui->pbAlfaPlay->setText("Play");
    return;
  }
}

void PCLViewer::on_pbRawPlay_clicked() {
  if (raw_playing_status) {
    ui->pbRawPlay->setText("Play");
    raw_playing_status = false;
    return;

  } else {
    ui->pbRawPlay->setText("Pause");
    raw_playing_status = true;
    return;
  }
}

void PCLViewer::on_pbAlfaPlay_clicked() {
  if (alfa_playing_status) {
    ui->pbAlfaPlay->setText("Play");
    alfa_playing_status = false;
    return;

  } else {
    ui->pbAlfaPlay->setText("Pause");
    alfa_playing_status = true;
    return;
  }
}

void PCLViewer::on_pb_select_bag_clicked() {
  QString filename =
      QFileDialog::getOpenFileName(this, tr("Open point cloud"), "", tr("ROS Bag(*.db3)"));
  if (filename.isEmpty()) return;

  ros_interface->load_bag(filename.toStdString());
}

void PCLViewer::on_pb_NextFrame_clicked() { ros_interface->publish_next_bag_pointcloud(); }

void PCLViewer::on_sb_FPS_valueChanged(double arg1) {
  if (arg1 > 0) {
    // m_ros_interface->fps = arg1;
    // m_ros_interface->update_bag_fps();
  }
}

void PCLViewer::on_pb_PlayBag_clicked() {
  if (ui->pb_PlayBag->text().contains("Restart")) {
    ros_interface->reload_bag();
    ui->pb_PlayBag->setText("Play");
    ros_interface->pause_bag();
  } else {
    if (ui->pb_PlayBag->text().contains("Play") && ros_interface->bag_loaded) {
      ui->pb_PlayBag->setText("Pause");
      ros_interface->play_bag(ui->sb_FPS->value());
    } else if (ui->pb_PlayBag->text().contains("Pause") && ros_interface->bag_loaded) {
      ui->pb_PlayBag->setText("Play");
      ros_interface->pause_bag();
    }
  }
}

void PCLViewer::on_rb_Loop_clicked() {
  // m_ros_interface->is_loop = ui->rb_Loop->isChecked();
  // m_ros_interface->loop_update_bag_index();
}

void PCLViewer::on_rb_Output_clicked() {
  if (ui->rb_Output->isChecked()) {
    ui->w_ExecFlow->show();
  } else {
    ui->w_ExecFlow->hide();
  }
}

void PCLViewer::on_cb_RemovePoints_clicked() {
  if (ui->cb_RemovePoints->isChecked()) {
    ui->l_ExecOrder->clear();
    ui->l_ExecOrder->addItem("Remove box Points");
    exec_order[0] = 1;
    exec_order[1] = 0;
    if (ui->cb_InjectNoise->isChecked()) {
      ui->l_ExecOrder->addItem("Noise injection");
      exec_order[1] = 2;
    }

  } else {
    exec_order[0] = 0;
    exec_order[1] = 0;
    ui->l_ExecOrder->clear();
    if (ui->cb_InjectNoise->isChecked()) {
      exec_order[0] = 2;
      ui->l_ExecOrder->addItem("Noise injection");
    }
  }
}

void PCLViewer::on_cb_InjectNoise_clicked() {
  if (ui->cb_InjectNoise->isChecked()) {
    ui->l_ExecOrder->clear();
    if (ui->cb_RemovePoints->isChecked()) {
      exec_order[0] = 1;
      exec_order[1] = 2;
      ui->l_ExecOrder->addItem("Remove box Points");
    } else {
      exec_order[0] = 2;
      exec_order[1] = 0;
    }
    ui->l_ExecOrder->addItem("Noise injection");

  } else {
    exec_order[0] = 0;
    exec_order[1] = 0;
    ui->l_ExecOrder->clear();
    if (ui->cb_RemovePoints->isChecked()) {
      exec_order[0] = 1;
      ui->l_ExecOrder->addItem("Remove box Points");
    }
  }
}

void PCLViewer::on_l_ExecOrder_currentItemChanged(QListWidgetItem *current,
                                                  QListWidgetItem *previous) {
  if (ui->l_ExecOrder->item(0)->text().compare("Remove box Points")) {
    if (ui->cb_InjectNoise->isChecked()) {
      exec_order[0] = 1;
      exec_order[1] = 2;
    } else {
      exec_order[0] = 1;
      exec_order[1] = 0;
    }

  } else if (ui->l_ExecOrder->item(0)->text().compare("Noise injection")) {
    if (ui->cb_RemovePoints->isChecked()) {
      exec_order[0] = 2;  // Execute noise injection first
      exec_order[1] = 1;  // Remove points
    } else {
      exec_order[0] = 1;  // Only aply noise injection
      exec_order[1] = 0;
    }
  }
}

void PCLViewer::on_lw_confignodes_itemDoubleClicked(QListWidgetItem *item) {
  ConfigWindow *newConfigWindow;
  newConfigWindow =
      new ConfigWindow(alfa_nodes[ui->lw_confignodes->currentRow()], ros_interface, this);
  newConfigWindow->setStyleSheet(combinear);
  newConfigWindow->show();
}

void PCLViewer::on_lw_metrics_itemDoubleClicked(QListWidgetItem *item) {
  MetricWindow *newMetricWindow =
      new MetricWindow(metric_topics[ui->lw_metrics->currentRow()], ros_interface, this);
  newMetricWindow->setStyleSheet(combinear);
  newMetricWindow->show();
}

void PCLViewer::on_cbHide_clicked() {
  if (ui->cbHide->text().contains("Hide")) {
    viewer_input->removeAllShapes();
    ui->qvtkWidget->renderWindow()->Render();
    ui->cbHide->setText("Show Boxes");
  } else {
    buildbox();
    ui->cbHide->setText("Hide Boxes");
  }
}

void PCLViewer::on_le_TopicName_textEdited(const QString &arg1) {}

void PCLViewer::on_pb_SendFrame_clicked() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr msg_cloud;
  msg_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::copyPointCloud(*raw_cloud, *msg_cloud);
  ros_interface->publish_point_cloud(msg_cloud, ui->le_TopicName->text().toStdString());
}

void PCLViewer::ros_spinner() {
  rclcpp::spin(ros_interface);
  rclcpp::shutdown();
}