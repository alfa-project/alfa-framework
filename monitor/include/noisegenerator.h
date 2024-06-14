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

#ifndef NOISEGENERATOR_H
#define NOISEGENERATOR_H

#include <QDialog>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include "boxsettings.h"
//#include "ros/ros.h"

using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
    class NoiseGenerator;
}

class NoiseGenerator : public QDialog
{
    Q_OBJECT

public:
    explicit NoiseGenerator(vector<box> *boxlist, pcl::PointCloud<PointT>::Ptr inputCloud, QWidget *parent = nullptr);
    void add_noise();
    void do_gausian(box target, pcl::PointXYZI center);
    void do_random(box target);
    pcl::PointXYZI calculate_center(box target);
    bool *outputToTopic;
    ~NoiseGenerator();

private slots:
    void on_pbAply_clicked();
    void on_rbGaussian_clicked();

signals:
    void showPCloud();

private:
    Ui::NoiseGenerator *ui;
    vector<box> *boxlist;
    pcl::PointCloud<PointT>::Ptr inputCloud;
    //ros::Publisher chatter_pub;
    unsigned int pcl2_Header_seq;
};

#endif // NOISEGENERATOR_H
