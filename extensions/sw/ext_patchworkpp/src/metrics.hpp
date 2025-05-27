#ifndef METRICS_H
#define METRICS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "alfa_msg/msg/alfa_metrics.hpp"
#include "math.h"

#define SENSOR_HEIGHT 1.73
const double VEGETATION_THR = -SENSOR_HEIGHT + 0.15;

/*------------------------------
            Labels
------------------------------*/

/* UNLABELED */
#define UNLABELED 0

/* OUTLIER */
#define OUTLIER 1

/* GROUND-RELATED */
#define ROAD 40
#define PARKING 44
#define SIDEWALK 48
#define OTHER_GROUND 49
#define LANE_MARKING 60

/* STRUCTURES */
#define BUILDING 50
#define OTHER_STRUCTURE 52

/* VEHICLE */
#define CAR 10
#define BICYCLE 11
#define BUS 13
#define MOTORCYCLE 15
#define ON_RAILS 16
#define TRUCK 18
#define OTHER_VEHICLE 20
#define MOVING_CAR 252
#define MOVING_ON_RAILS 256
#define MOVING_BUS 257
#define MOVING_TRUCK 258
#define MOVING_OTHER_VEHICLE 259

/* NATURE */
#define VEGETATION 70
#define TRUNK 71
#define TERRAIN 72

/* HUMAN */
#define PERSON 30
#define BICYCLIST 31
#define MOTORCYCLIST 32
#define MOVING_BICYCLIST 253
#define MOVING_PERSON 254
#define MOVING_MOTORCYCLIST 255

/* OBJECT */
#define FENCE 51
#define POLE 80
#define TRAFFIC_SIGN 81
#define OTHER_OBJECT 99

/*----------------------------*/

/*------------------------------
            Classes
------------------------------*/

std::vector<int> outlier_class = {UNLABELED, OUTLIER};

std::vector<int> ground_class = {ROAD, PARKING, SIDEWALK, OTHER_GROUND, LANE_MARKING};
std::vector<int> ground_class_with_terrain = {ROAD,         PARKING,      SIDEWALK,
                                              OTHER_GROUND, LANE_MARKING, TERRAIN};
std::vector<int> ground_class_with_terrain_and_veg = {
    ROAD, PARKING, SIDEWALK, OTHER_GROUND, LANE_MARKING, TERRAIN, VEGETATION};
std::vector<int> traversable_ground_class = {ROAD, PARKING, LANE_MARKING, OTHER_GROUND};

std::vector<int> structure_class = {BUILDING, OTHER_STRUCTURE};

std::vector<int> vehicle_class = {CAR,        MOVING_CAR,   BICYCLE,       BUS,
                                  MOVING_BUS, MOTORCYCLE,   ON_RAILS,      MOVING_ON_RAILS,
                                  TRUCK,      MOVING_TRUCK, OTHER_VEHICLE, MOVING_OTHER_VEHICLE};

std::vector<int> nature_class = {VEGETATION, TRUNK, TERRAIN};

std::vector<int> human_class = {PERSON,           MOVING_PERSON, BICYCLIST,
                                MOVING_BICYCLIST, MOTORCYCLIST,  MOVING_MOTORCYCLIST};

std::vector<int> object_class = {FENCE, POLE, TRAFFIC_SIGN, OTHER_OBJECT};

std::vector<int> all_classes = {UNLABELED,
                                OUTLIER,
                                ROAD,
                                PARKING,
                                SIDEWALK,
                                OTHER_GROUND,
                                LANE_MARKING,
                                BUILDING,
                                OTHER_STRUCTURE,
                                CAR,
                                MOVING_CAR,
                                BICYCLE,
                                BUS,
                                MOVING_BUS,
                                MOTORCYCLE,
                                ON_RAILS,
                                MOVING_ON_RAILS,
                                TRUCK,
                                MOVING_TRUCK,
                                OTHER_VEHICLE,
                                MOVING_OTHER_VEHICLE,
                                VEGETATION,
                                TRUNK,
                                TERRAIN,
                                PERSON,
                                MOVING_PERSON,
                                BICYCLIST,
                                MOVING_BICYCLIST,
                                MOTORCYCLIST,
                                MOVING_MOTORCYCLIST,
                                FENCE,
                                POLE,
                                TRAFFIC_SIGN,
                                OTHER_OBJECT};

struct label {
  int val;
  const char* msg;
  const char* cls;
};

struct label labels[] = {{UNLABELED, "Unlabeled", "Unlabeled"},
                         {OUTLIER, "Outlier", "Outlier"},
                         {ROAD, "Road", "Ground"},
                         {PARKING, "Parking", "Ground"},
                         {SIDEWALK, "Sidewalk", "Ground"},
                         {OTHER_GROUND, "Other ground", "Ground"},
                         {LANE_MARKING, "Lane marking", "Ground"},
                         {BUILDING, "Building", "Structure"},
                         {OTHER_STRUCTURE, "Other structure", "Structure"},
                         {CAR, "Car", "Vehicle"},
                         {MOVING_CAR, "Moving car", "Vehicle"},
                         {BICYCLE, "Bicycle", "Vehicle"},
                         {BUS, "Bus", "Vehicle"},
                         {MOVING_BUS, "Moving bus", "Vehicle"},
                         {MOTORCYCLE, "Motorcycle", "Vehicle"},
                         {ON_RAILS, "On rails", "Vehicle"},
                         {MOVING_ON_RAILS, "Moving on rails", "Vehicle"},
                         {TRUCK, "Truck", "Vehicle"},
                         {MOVING_TRUCK, "Moving truck", "Vehicle"},
                         {OTHER_VEHICLE, "Other vehicle", "Vehicle"},
                         {MOVING_OTHER_VEHICLE, "Moving other vehicle", "Vehicle"},
                         {VEGETATION, "Vegetation", "Nature"},
                         {TRUNK, "Trunk", "Nature"},
                         {TERRAIN, "Terrain", "Nature"},
                         {PERSON, "Person", "Human"},
                         {MOVING_PERSON, "Moving person", "Human"},
                         {BICYCLIST, "Bicyclist", "Human"},
                         {MOVING_BICYCLIST, "Moving bicyclist", "Human"},
                         {MOTORCYCLIST, "Motorcyclist", "Human"},
                         {MOVING_MOTORCYCLIST, "Moving motorcyclist", "Human"},
                         {FENCE, "Fence", "Object"},
                         {POLE, "Pole", "Object"},
                         {TRAFFIC_SIGN, "Traffic sign", "Object"},
                         {OTHER_OBJECT, "Other object", "Object"}};

struct user_defined {  // should have units in order to print
  float value;
  string name;
};

using namespace std;

template <typename PointT>
class Metrics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Metrics(int no_user_defined = 0, string user_defined_1 = "", string user_defined_2 = "",
          string user_defined_3 = "", string user_defined_4 = "", string user_defined_5 = "",
          string user_defined_6 = "", string user_defined_7 = "");

  ~Metrics();

  void calculate_metrics(pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> cloud_ground,
                         pcl::PointCloud<PointT> cloud_nonground, float user_1 = 0,
                         float user_2 = 0, float user_3 = 0, float user_4 = 0, float user_5 = 0,
                         float user_6 = 0, float user_7 = 0);

  void callback_shutdown();

  void post_processing(alfa_msg::msg::MetricMessage handler_time,
                       alfa_msg::msg::MetricMessage full_time);

 private:
  float mean_vector(vector<float> v) { return std::accumulate(v.begin(), v.end(), 0.0) / v.size(); }
  float std_dev_vector(vector<float> v, double vmean) {
    return std::sqrt(
        std::accumulate(v.begin(), v.end(), 0.0,
                        [&](double acc, double x) { return acc + std::pow(x - vmean, 2); }) /
        v.size());
  }

  int count_num_ground(const pcl::PointCloud<PointT>& pc);
  void publish_output_metrics();
  void get_classes_in_cloud(pcl::PointCloud<PointT> cloud_in, bool select);

  std::vector<pair<int, int>> count_num_each_class(const pcl::PointCloud<PointT> pc);
  std::vector<pair<int, int>> set_initial_gt_counts(std::vector<int>& gt_classes);

  /*
      VARIABLES
  */

  int frames;
  double _TP, _FP, _TN, _FN;
  float total_points_cloud;
  float total_points_out;
  float removed_points;

  alfa_msg::msg::MetricMessage true_positive_rate;
  alfa_msg::msg::MetricMessage true_negative_rate;
  alfa_msg::msg::MetricMessage positive_predictive_value;
  alfa_msg::msg::MetricMessage negative_predictive_value;
  alfa_msg::msg::MetricMessage f1;
  alfa_msg::msg::MetricMessage true_positive_rate_mean;
  alfa_msg::msg::MetricMessage true_negative_rate_mean;
  alfa_msg::msg::MetricMessage positive_predictive_value_mean;
  alfa_msg::msg::MetricMessage negative_predictive_value_mean;
  alfa_msg::msg::MetricMessage f1_mean;
  alfa_msg::msg::MetricMessage true_positive_rate_dev;
  alfa_msg::msg::MetricMessage true_negative_rate_dev;
  alfa_msg::msg::MetricMessage positive_predictive_value_dev;
  alfa_msg::msg::MetricMessage negative_predictive_value_dev;
  alfa_msg::msg::MetricMessage f1_dev;
  alfa_msg::msg::MetricMessage total_points_mean;
  alfa_msg::msg::MetricMessage points_removed_mean;
  alfa_msg::msg::MetricMessage TP_mean;
  alfa_msg::msg::MetricMessage FP_mean;
  alfa_msg::msg::MetricMessage TN_mean;
  alfa_msg::msg::MetricMessage FN_mean;

  alfa_msg::msg::MetricMessage fr;
  alfa_msg::msg::MetricMessage TP;
  alfa_msg::msg::MetricMessage FP;
  alfa_msg::msg::MetricMessage TN;
  alfa_msg::msg::MetricMessage FN;
  alfa_msg::msg::MetricMessage total_points;
  alfa_msg::msg::MetricMessage points_removed;

  vector<float> tpr_vec;
  vector<float> tnr_vec;
  vector<float> ppv_vec;
  vector<float> npv_vec;
  vector<float> f1_vec;
  vector<float> acc_vec;
  vector<float> ioug_vec;
  vector<float> total_points_vec;
  vector<float> total_points_out_vec;
  vector<float> removed_vec;
  vector<float> tp_vec;
  vector<float> fp_vec;
  vector<float> tn_vec;
  vector<float> fn_vec;
  vector<float> htime_vec;
  vector<float> ptime_vec;

  int no_usr_defined_vec;
  user_defined user_defined_1;
  user_defined user_defined_2;
  user_defined user_defined_3;
  user_defined user_defined_4;
  user_defined user_defined_5;
  user_defined user_defined_6;
  user_defined user_defined_7;
  vector<float> user_defined_vec_1;
  vector<float> user_defined_vec_2;
  vector<float> user_defined_vec_3;
  vector<float> user_defined_vec_4;
  vector<float> user_defined_vec_5;
  vector<float> user_defined_vec_6;
  vector<float> user_defined_vec_7;

  int total_points_ground;
  int total_points_non_ground;
  vector<pair<int, int>> total_classes_ground;
  vector<pair<int, int>> total_classes_non_ground;
};

template <typename PointT>
inline Metrics<PointT>::Metrics(int no_usr_defined, string user_name_1, string user_name_2,
                                string user_name_3, string user_name_4, string user_name_5,
                                string user_name_6, string user_name_7) {
  no_usr_defined_vec = no_usr_defined;

  switch (no_usr_defined_vec) {
    case 7:
      user_defined_7.name = user_name_7;

    case 6:
      user_defined_6.name = user_name_6;

    case 5:
      user_defined_5.name = user_name_5;

    case 4:
      user_defined_4.name = user_name_4;

    case 3:
      user_defined_3.name = user_name_3;

    case 2:
      user_defined_2.name = user_name_2;

    case 1:
      user_defined_1.name = user_name_1;
      break;

    default:
      break;
  }

  //////////////////////////////////////////////////////

  fr.units = "frame(s)";
  fr.metric_name = "Number of frames";
  fr.metric = 0;

  TP.units = "points";
  TP.metric_name = "TP";
  TP.metric = 0;

  TP_mean.units = "points";
  TP_mean.metric_name = "TP Mean";
  TP_mean.metric = 0;

  FP.units = "points";
  FP.metric_name = "FP";
  FP.metric = 0;

  FP_mean.units = "points";
  FP_mean.metric_name = "FP Mean";
  FP_mean.metric = 0;

  TN.units = "points";
  TN.metric_name = "TN";
  TN.metric = 0;

  TN_mean.units = "points";
  TN_mean.metric_name = "TN Mean";
  TN_mean.metric = 0;

  FN.units = "points";
  FN.metric_name = "FN";
  FN.metric = 0;

  FN_mean.units = "points";
  FN_mean.metric_name = "FN Mean";
  FN_mean.metric = 0;

  //////////////////////////////////////////////////////

  total_points.units = "points";
  total_points.metric_name = "Points in cloud";
  total_points.metric = 0;

  total_points_mean.units = "points";
  total_points_mean.metric_name = "Points in cloud Mean";
  total_points_mean.metric = 0;

  //////////////////////////////////////////////////////

  points_removed.units = "points";
  points_removed.metric_name = "Points removed from cloud";
  points_removed.metric = 0;

  points_removed_mean.units = "points";
  points_removed_mean.metric_name = "Points removed from cloud Mean";
  points_removed_mean.metric = 0;

  //////////////////////////////////////////////////////

  true_positive_rate.units = "%";
  true_positive_rate.metric_name = "TPR : True Positive Rate // Sensitivity // Recall";
  true_positive_rate.metric = 0;

  true_positive_rate_mean.units = "%";
  true_positive_rate_mean.metric_name = "TPR Mean";
  true_positive_rate_mean.metric = 0;

  true_positive_rate_dev.units = "%";
  true_positive_rate_dev.metric_name = "TPR Std Dev";
  true_positive_rate_dev.metric = 0;

  //////////////////////////////////////////////////////

  true_negative_rate.units = "%";
  true_negative_rate.metric_name = "TNR : True Negative Rate // Specificity";
  true_negative_rate.metric = 0;

  true_negative_rate_mean.units = "%";
  true_negative_rate_mean.metric_name = "TNR Mean";
  true_negative_rate_mean.metric = 0;

  true_negative_rate_dev.units = "%";
  true_negative_rate_dev.metric_name = "TNR Std Dev";
  true_negative_rate_dev.metric = 0;

  //////////////////////////////////////////////////////

  positive_predictive_value.units = "%";
  positive_predictive_value.metric_name = "PPV : Positive Predictive Value // Precision";
  positive_predictive_value.metric = 0;

  positive_predictive_value_mean.units = "%";
  positive_predictive_value_mean.metric_name = "PPV Mean";
  positive_predictive_value_mean.metric = 0;

  positive_predictive_value_dev.units = "%";
  positive_predictive_value_dev.metric_name = "PPV Std Dev";
  positive_predictive_value_dev.metric = 0;

  //////////////////////////////////////////////////////

  negative_predictive_value.units = "%";
  negative_predictive_value.metric_name = "NPV : Negative Predictive Value";
  negative_predictive_value.metric = 0;

  negative_predictive_value_mean.units = "%";
  negative_predictive_value_mean.metric_name = "NPV Mean";
  negative_predictive_value_mean.metric = 0;

  negative_predictive_value_dev.units = "%";
  negative_predictive_value_dev.metric_name = "NPV Std Dev";
  negative_predictive_value_dev.metric = 0;

  //////////////////////////////////////////////////////

  f1.units = "%";
  f1.metric_name = "F1-Score";
  f1.metric = 0;

  f1_mean.units = "%";
  f1_mean.metric_name = "F1-Score Mean";
  f1_mean.metric = 0;

  f1_dev.units = "%";
  f1_dev.metric_name = "F1-Score Std Dev";
  f1_dev.metric = 0;

  //////////////////////////////////////////////////////

  frames = 0;
  total_points_cloud = 0;
  removed_points = 0;
}

template <typename PointT>
inline Metrics<PointT>::~Metrics() {}

template <typename PointT>
inline void Metrics<PointT>::calculate_metrics(pcl::PointCloud<PointT> cloud_in,
                                               pcl::PointCloud<PointT> cloud_ground,
                                               pcl::PointCloud<PointT> cloud_nonground,
                                               float user_1, float user_2, float user_3,
                                               float user_4, float user_5, float user_6,
                                               float user_7) {
  frames++;

  std::cout << "\r\e[K"
            << "Point cloud [" << frames << "] received!!" << std::flush;

  // int eTP = count_num_ground(cloud_ground);                 // Nº pontos de chão na cloud de chão
  // - True Positives int eFP = cloud_ground.size() - eTP;  // False Positives

  // int eFN = count_num_ground(cloud_nonground);
  // int eTN = cloud_nonground.size() - eFN;

  // _TP = eTP;
  // _FP = eFP;
  // _TN = eTN;
  // _FN = eFN;

  // total_points_cloud = cloud_in.size();
  // removed_points = cloud_ground.size();

  // if(no_usr_defined_vec > 0)
  // {
  //     user_defined_1.value = user_1;
  //     user_defined_2.value = user_2;
  //     user_defined_3.value = user_3;
  //     user_defined_4.value = user_4;
  //     user_defined_5.value = user_5;
  //     user_defined_6.value = user_6;
  //     user_defined_7.value = user_7;
  // }

  // get_classes_in_cloud(cloud_ground, false);
  // get_classes_in_cloud(cloud_nonground, true);
  // publish_output_metrics();
}

template <typename PointT>
inline void Metrics<PointT>::callback_shutdown() {
  if (frames > 0) {
    double ht_mean = mean_vector(htime_vec);
    double ht_dev = std_dev_vector(htime_vec, ht_mean);
    double ft_mean = mean_vector(ptime_vec);
    double ft_dev = std_dev_vector(ptime_vec, ft_mean);

    // double total_mean = mean_vector(total_points_vec);
    // double total_dev = std_dev_vector(total_points_vec, total_mean);
    // double totalo_mean = mean_vector(total_points_out_vec);
    // double totalo_dev = std_dev_vector(total_points_out_vec, totalo_mean);
    // double removed_mean = mean_vector(removed_vec);
    // double removed_dev = std_dev_vector(removed_vec, removed_mean);

    // double usr1_mean = mean_vector(user_defined_vec_1);
    // double usr1_dev = std_dev_vector(user_defined_vec_1, usr1_mean);
    // double usr2_mean = mean_vector(user_defined_vec_2);
    // double usr2_dev = std_dev_vector(user_defined_vec_2, usr2_mean);

    // double _tpr_mean = mean_vector(tpr_vec);
    // double _tpr_dev = std_dev_vector(tpr_vec, _tpr_mean);
    // double _tnr_mean = mean_vector(tnr_vec);
    // double _tnr_dev = std_dev_vector(tnr_vec, _tnr_mean);
    // double _ppv_mean = mean_vector(ppv_vec);
    // double _ppv_dev = std_dev_vector(ppv_vec, _ppv_mean);
    // double _npv_mean = mean_vector(npv_vec);
    // double _npv_dev = std_dev_vector(npv_vec, _npv_mean);
    // double _f1_mean = mean_vector(f1_vec);
    // double _f1_dev = std_dev_vector(f1_vec, _f1_mean);
    // double _acc_mean = mean_vector(acc_vec);
    // double _acc_dev = std_dev_vector(acc_vec, _acc_mean);
    // double _ioug_mean = mean_vector(ioug_vec);
    // double _ioug_dev = std_dev_vector(ioug_vec, _ioug_mean);

    std::cout << std::endl << "------ METRICS -------" << std::endl;
    std::cout << "Number of frames: " << frames << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << "TP: " << (int)mean_vector(tp_vec) << " +/- " << (int)std_dev_vector(tp_vec,
    // mean_vector(tp_vec)) << std::endl; std::cout << "FP: " << (int)mean_vector(fp_vec) << " +/- "
    // << (int)std_dev_vector(fp_vec, mean_vector(fp_vec)) << std::endl; std::cout << "TN: " <<
    // (int)mean_vector(tn_vec) << " +/- " << (int)std_dev_vector(tn_vec, mean_vector(tn_vec)) <<
    // std::endl; std::cout << "FN: " << (int)mean_vector(fn_vec) << " +/- " <<
    // (int)std_dev_vector(fn_vec, mean_vector(fn_vec)) << std::endl; std::cout <<
    // "----------------------" << std::endl; std::cout.precision(4); std::cout << "Recall: " <<
    // _tpr_mean * 100 << " +/- " << _tpr_dev * 100 << " %" << std::endl; std::cout << "Precision: "
    // << _ppv_mean * 100 << " +/- " << _ppv_dev * 100 << " %" << std::endl; std::cout << "F1-Score:
    // " << _f1_mean * 100 << " +/- " << _f1_dev * 100 << " %" << std::endl; std::cout << "Accuracy:
    // " << _acc_mean * 100 << " +/- " << _acc_dev * 100 << " %" << std::endl; std::cout << "IoUg: "
    // << _ioug_mean * 100 << " +/- " << _ioug_dev * 100 << " %" << std::endl; std::cout <<
    // "----------------------" << std::endl; std::cout.precision(6); std::cout << "Total points
    // [In]: " << total_mean << " +/- " << total_dev << std::endl; std::cout << "Total points [Out]:
    // " << totalo_mean << " +/- " << totalo_dev << std::endl; std::cout << "Removed points: " <<
    // removed_mean << " +/- " << removed_dev << std::endl; std::cout << "----------------------" <<
    // std::endl;
    std::cout.precision(4);
    std::cout << "Handler Time: " << ht_mean / 1000 << " +/- " << ht_dev / 1000 << " ms"
              << std::endl;
    std::cout << "Full Processing Time: " << ft_mean / 1000 << " +/- " << ft_dev / 1000 << " ms"
              << std::endl;
    std::cout << "-----------------------" << std::endl;

    // switch (no_usr_defined_vec)
    // {
    //    case 6: std::cout << user_defined_6.name << ": " <<
    //    static_cast<int>(std::accumulate(user_defined_vec_6.begin(), user_defined_vec_6.end(),
    //    0.0) / user_defined_vec_6.size()) << "" << std::endl;

    //    case 5: std::cout << user_defined_5.name << ": "<<
    //    static_cast<int>(std::accumulate(user_defined_vec_5.begin(), user_defined_vec_5.end(),
    //    0.0) / user_defined_vec_5.size()) << "" << std::endl;

    //    case 4: std::cout << user_defined_4.name << ": "<<
    //    static_cast<int>(std::accumulate(user_defined_vec_4.begin(), user_defined_vec_4.end(),
    //    0.0) / user_defined_vec_4.size()) << "" << std::endl;

    //    case 3: std::cout << user_defined_3.name << ": "<<
    //    static_cast<int>(std::accumulate(user_defined_vec_3.begin(), user_defined_vec_3.end(),
    //    0.0) / user_defined_vec_3.size()) << "" << std::endl;

    //    case 2: std::cout << user_defined_2.name << ": "<<
    //    static_cast<int>(std::accumulate(user_defined_vec_2.begin(), user_defined_vec_2.end(),
    //    0.0) / user_defined_vec_2.size())<< "" << std::endl;

    //    case 1: std::cout << user_defined_1.name << ": "<<
    //    static_cast<int>(std::accumulate(user_defined_vec_1.begin(), user_defined_vec_1.end(),
    //    0.0) / user_defined_vec_1.size())<< "" << std::endl;
    //            break;

    //    default:break;
    // }
    // std::cout << "----------------------" << std::endl;

    // std::cout << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << "This is ground!" << std::endl;
    // std::cout << "Number of frames: " << frames << std::endl;
    // std::cout << "Number of points: " << total_points_ground / frames << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << "\t|   Class   |" << "       |            Subclass            |" << std::endl <<
    // std::endl;

    // for (auto it : total_classes_ground)
    // {
    //     for (int i = 0; i < sizeof(labels)/sizeof(labels[0]); i++)
    //     {
    //         if(it.first == labels[i].val)
    //         {
    //             if(i > 0 && labels[i].cls != labels[i-1].cls)
    //             {
    //                 std::cout << std::endl;
    //             }

    //             std::cout << "\t"
    //                     << left
    //                     << setfill(' ')
    //                     << std::setw(20)
    //                     << labels[i].cls
    //                     << right
    //                     << setfill(' ')
    //                     << labels[i].msg
    //                     << " ("
    //                     << it.first    // string (key)
    //                     << ") has "
    //                     << (it.second / frames)   // string's value
    //                     << " points"
    //                     << std::endl << std::flush;
    //             continue;
    //         }
    //     }
    // }

    // std::cout << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << "This is non ground!" << std::endl;
    // std::cout << "Number of frames: " << frames << std::endl;
    // std::cout << "Number of points: " << total_points_non_ground / frames << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << "\t|   Class   |" << "       |            Subclass            |" << std::endl <<
    // std::endl;

    // for (auto it : total_classes_non_ground)
    // {
    //     for (int i = 0; i < sizeof(labels)/sizeof(labels[0]); i++)
    //     {
    //         if(it.first == labels[i].val)
    //         {
    //             if(i > 0 && labels[i].cls != labels[i-1].cls)
    //             {
    //                 std::cout << std::endl;
    //             }

    //             std::cout << "\t"
    //                     << left
    //                     << setfill(' ')
    //                     << std::setw(20)
    //                     << labels[i].cls
    //                     << right
    //                     << setfill(' ')
    //                     << labels[i].msg
    //                     << " ("
    //                     << it.first    // string (key)
    //                     << ") has "
    //                     << (it.second / frames)   // string's value
    //                     << " points"
    //                     << std::endl << std::flush;
    //             continue;
    //         }
    //     }
    // }
  } else
    std::cout << std::endl << "No frames were processed yet!" << std::endl;
}

template <typename PointT>
inline void Metrics<PointT>::post_processing(alfa_msg::msg::MetricMessage handler_time,
                                             alfa_msg::msg::MetricMessage full_time) {
  htime_vec.push_back(handler_time.metric);
  ptime_vec.push_back(full_time.metric);
}

template <typename PointT>
inline void Metrics<PointT>::publish_output_metrics() {
  double _tpr = _TP / (_TP + _FN);
  double _tnr = _TN / (_TN + _FP);
  double _ppv = _TP / (_TP + _FP);
  double _npv = _TN / (_TN + _FN);
  float _f1 = 2 * (_ppv * _tpr) / (_ppv + _tpr);
  float _acc = (_TP + _TN) / (_TP + _TN + _FP + _FN);
  float _ioug = _TP / (_TP + _FP + _FN);

  tpr_vec.push_back(_tpr);
  tnr_vec.push_back(_tnr);
  ppv_vec.push_back(_ppv);
  npv_vec.push_back(_npv);
  f1_vec.push_back(_f1);
  acc_vec.push_back(_acc);
  ioug_vec.push_back(_ioug);
  tp_vec.push_back(_TP);
  fp_vec.push_back(_FP);
  tn_vec.push_back(_TN);
  fn_vec.push_back(_FN);

  total_points_vec.push_back(static_cast<float>(total_points_cloud));
  total_points_out_vec.push_back(static_cast<float>(total_points_out));
  removed_vec.push_back(static_cast<float>(removed_points));

  // switch (no_usr_defined_vec)
  // {
  //     case 7: user_defined_vec_7.push_back(user_defined_7.value);

  //     case 6: user_defined_vec_6.push_back(user_defined_6.value);

  //     case 5: user_defined_vec_5.push_back(user_defined_5.value);

  //     case 4: user_defined_vec_4.push_back(user_defined_4.value);

  //     case 3: user_defined_vec_3.push_back(user_defined_3.value);

  //     case 2: user_defined_vec_2.push_back(user_defined_2.value);

  //     case 1: user_defined_vec_1.push_back(user_defined_1.value);
  //             break;

  //     default:break;
  // }

  // double _tpr_mean = mean_vector(tpr_vec);
  // double _tpr_dev = std_dev_vector(tpr_vec, _tpr_mean);
  // double _tnr_mean = mean_vector(tnr_vec);
  // double _tnr_dev = std_dev_vector(tnr_vec, _tnr_mean);
  // double _ppv_mean = mean_vector(ppv_vec);
  // double _ppv_dev = std_dev_vector(ppv_vec, _ppv_mean);
  // double _npv_mean = mean_vector(npv_vec);
  // double _npv_dev = std_dev_vector(npv_vec, _npv_mean);
  // double _f1_mean = mean_vector(f1_vec);
  // double _f1_dev = std_dev_vector(f1_vec, _f1_mean);

  // double _tpoints_mean = mean_vector(total_points_vec);
  // double _rpoints_mean = mean_vector(removed_vec);

  // fr.metric = frames;

  // // Metrics of last frame

  // true_positive_rate.metric = _tpr * 100;
  // true_negative_rate.metric = _tnr * 100;
  // positive_predictive_value.metric = _ppv * 100;
  // negative_predictive_value.metric = _npv * 100;
  // f1.metric = _f1 * 100;

  // TP.metric = _TP;
  // FP.metric = _FP;
  // TN.metric = _TN;
  // FN.metric = _FN;

  // TP_mean.metric = mean_vector(tp_vec);
  // FP_mean.metric = mean_vector(fp_vec);
  // TN_mean.metric = mean_vector(tn_vec);
  // FN_mean.metric = mean_vector(fn_vec);

  // total_points.metric = total_points_cloud;
  // points_removed.metric = removed_points;

  // // Metrics mean and std dev of all frames

  // true_positive_rate_mean.metric = _tpr_mean * 100;
  // true_negative_rate_mean.metric = _tnr_mean * 100;
  // positive_predictive_value_mean.metric = _ppv_mean * 100;
  // negative_predictive_value_mean.metric = _npv_mean * 100;
  // f1_mean.metric = _f1_mean * 100;
  // total_points_mean.metric = _tpoints_mean * 100;
  // points_removed_mean.metric = _rpoints_mean * 100;

  // true_positive_rate_dev.metric = _tpr_dev * 100;
  // true_negative_rate_dev.metric = _tnr_dev * 100;
  // positive_predictive_value_dev.metric = _ppv_dev * 100;
  // negative_predictive_value_dev.metric = _npv_dev * 100;
  // f1_dev.metric = _f1_dev * 100;
}

template <typename PointT>
inline void Metrics<PointT>::get_classes_in_cloud(pcl::PointCloud<PointT> cloud_in, bool select) {
  vector<pair<int, int>> classes;
  int points_in, points_out;
  points_in = cloud_in.points.size();

  classes = count_num_each_class(cloud_in);

  points_out = 0;
  if (select) {
    if (total_classes_non_ground.empty()) {
      for (const auto it : classes) {
        total_classes_non_ground.push_back(it);
        points_out += it.second;
      }

    } else {
      for (int i = 0; i < total_classes_non_ground.size(); i++) {
        total_classes_non_ground[i].second += classes[i].second;
        points_out += classes[i].second;
      }
    }
  } else {
    if (total_classes_ground.empty()) {
      for (const auto it : classes) {
        total_classes_ground.push_back(it);
        points_out += it.second;
      }

    } else {
      for (int i = 0; i < total_classes_ground.size(); i++) {
        total_classes_ground[i].second += classes[i].second;
        points_out += classes[i].second;
      }
    }
  }

  // std::cout << endl << "\tPoints in cloud = " << points_in << endl;
  assert(points_in == points_out);

  if (select)
    total_points_non_ground += points_out;
  else
    total_points_ground += points_out;
}

template <typename PointT>
inline int Metrics<PointT>::count_num_ground(const pcl::PointCloud<PointT>& pc) {
  int num_ground = 0;

  std::vector<int>::iterator iter;

  for (auto const& pt : pc.points) {
    iter = std::find(ground_class_with_terrain_and_veg.begin(),
                     ground_class_with_terrain_and_veg.end(), (uint16_t)pt.custom_field);
    if (iter !=
        ground_class_with_terrain_and_veg.end()) {  // corresponding class is in ground classes
      if ((uint16_t)pt.custom_field == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          num_ground++;
        }
      } else
        num_ground++;
    }
  }
  return num_ground;
}

template <typename PointT>
inline std::vector<pair<int, int>> Metrics<PointT>::set_initial_gt_counts(
    std::vector<int>& gt_classes) {
  vector<pair<int, int>> gt_counts;

  for (auto it : gt_classes) {
    gt_counts.push_back(pair<int, int>(it, 0));
  }

  return gt_counts;
}

template <typename PointT>
inline std::vector<pair<int, int>> Metrics<PointT>::count_num_each_class(
    const pcl::PointCloud<PointT> pc) {
  auto gt_counts = set_initial_gt_counts(all_classes);
  std::vector<int>::iterator iter;

  for (auto const pt : pc.points) {
    iter = std::find(all_classes.begin(), all_classes.end(), (uint16_t)pt.custom_field);
    if (iter != all_classes.end()) {  // corresponding class is in all_classes
      for (auto& i : gt_counts) {
        if (i.first == (uint16_t)pt.custom_field) {
          i.second++;
          continue;
        }
      }
    }
  }
  return gt_counts;
}

#endif