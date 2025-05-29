/**
 * @file patchworkpp.hpp
 * @author Seungjae Lee
 * @brief
 * @version 0.1
 * @date 2022-07-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#include <pcl/common/centroid.h>
#include <time.h>

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <chrono>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <variant>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55
#define FLAT_ENOUGH 0.2
#define TOO_HIGH_ELEVATION 0.0
#define TOO_TILTED 1.0

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using namespace std;

/*
    @brief PathWork ROS Node.
*/
template <typename PointTP>
bool point_z_cmp(PointTP a, PointTP b) {
  return a.z < b.z;
}

template <typename PointTP>
struct RevertCandidate {
  int concentric_idx;
  int sector_idx;
  double ground_flatness;
  double line_variable;
  Eigen::Vector4f pc_mean;
  pcl::PointCloud<PointTP> regionwise_ground;

  RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var,
                  Eigen::Vector4f _pc_mean, pcl::PointCloud<PointTP> _ground)
      : concentric_idx(_c_idx),
        sector_idx(_s_idx),
        ground_flatness(_flatness),
        line_variable(_line_var),
        pc_mean(_pc_mean),
        regionwise_ground(_ground) {}
};

template <typename PointTP>
class PatchWorkpp {
 public:
  typedef std::vector<pcl::PointCloud<PointTP>> Ring;
  typedef std::vector<Ring> Zone;

  PatchWorkpp() {
    // Init ROS related
    // ROS_INFO("Inititalizing PatchWork++...");

    variableMap = {{"verbose_", &verbose_},
                   {"enable_RNR_", &enable_RNR_},
                   {"enable_RVPF_", &enable_RVPF_},
                   {"enable_TGR_", &enable_TGR_},
                   {"num_iter_", &num_iter_},
                   {"num_lpr_", &num_lpr_},
                   {"num_min_pts_", &num_min_pts_},
                   {"num_zones_", &num_zones_},
                   {"num_rings_of_interest_", &num_rings_of_interest_},
                   {"sensor_height_", &sensor_height_},
                   {"th_seeds_", &th_seeds_},
                   {"th_dist_", &th_dist_},
                   {"th_seeds_v_", &th_seeds_v_},
                   {"th_dist_v_", &th_dist_v_},
                   {"max_range_", &max_range_},
                   {"min_range_", &min_range_},
                   {"uprightness_thr_", &uprightness_thr_},
                   {"adaptive_seed_selection_margin_", &adaptive_seed_selection_margin_},
                   {"min_range_z2_", &min_range_z2_},
                   {"min_range_z3_", &min_range_z3_},
                   {"min_range_z4_", &min_range_z4_},
                   {"RNR_ver_angle_thr_", &RNR_ver_angle_thr_},
                   {"RNR_intensity_thr_", &RNR_intensity_thr_},
                   {"max_flatness_storage_", &max_flatness_storage_},
                   {"max_elevation_storage_", &max_elevation_storage_}};

    visualize_ = false;
  }

  void initialize() {
    // CZM denotes 'Concentric Zone Model'. Please refer to our paper
    num_sectors_each_zone_ = {16, 32, 54, 32};
    num_rings_each_zone_ = {2, 4, 4, 4};
    elevation_thr_ = {0.0, 0.0, 0.0, 0.0};
    flatness_thr_ = {0.0, 0.0, 0.0, 0.0};

    if (num_zones_ != 4 || num_sectors_each_zone_.size() != num_rings_each_zone_.size()) {
      throw invalid_argument(
          "Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone");
    }
    if (elevation_thr_.size() != flatness_thr_.size()) {
      throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
    }

    visualize_ = false;

    revert_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

    min_ranges_ = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
    ring_sizes_ = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                   (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                   (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                   (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    sector_sizes_ = {
        2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
        2 * M_PI / num_sectors_each_zone_.at(2), 2 * M_PI / num_sectors_each_zone_.at(3)};

    for (int i = 0; i < num_zones_; i++) {
      Zone z;
      initialize_zone(z, num_sectors_each_zone_[i], num_rings_each_zone_[i]);
      ConcentricZoneModel_.push_back(z);
    }
  }

  void estimate_ground(pcl::PointCloud<PointTP> cloud_in, pcl::PointCloud<PointTP> &cloud_ground,
                       pcl::PointCloud<PointTP> &cloud_nonground, double &time_taken_RNR,
                       double &time_taken_CZM, double &time_taken_SORT,
                       double &time_taken_GROUND_ESTIMATE, double &time_taken_AGLE,
                       double &time_taken_TGR, double &time_taken_UPDATE);

  void update_parameters(const string parameter_name, const double parameter_value) {
    auto iter = variableMap.find(parameter_name);
    if (iter != variableMap.end()) {
      auto &value = iter->second;
      if (std::holds_alternative<int *>(value)) {
        *(std::get<int *>(value)) = static_cast<int>(parameter_value);
      } else if (std::holds_alternative<double *>(value)) {
        *(std::get<double *>(value)) = parameter_value;
      } else if (std::holds_alternative<bool *>(value)) {
        *(std::get<bool *>(value)) = static_cast<bool>(parameter_value);
      }
    }
  }

  void print_parameters(const string parameter_name) {
    auto iter = variableMap.find(parameter_name);
    if (iter != variableMap.end()) {
      auto &value = iter->second;
      if (std::holds_alternative<int *>(value)) {
        std::cout << *(std::get<int *>(value)) << std::endl;
      } else if (std::holds_alternative<double *>(value)) {
        std::cout << *(std::get<double *>(value)) << std::endl;
      } else if (std::holds_alternative<bool *>(value)) {
        std::cout << *(std::get<bool *>(value)) << std::endl;
      }
    }
  }

 private:
  // Every private member variable is written with the undescore("_") in its end.

  std::recursive_mutex mutex_;

  int num_iter_;
  int num_lpr_;
  int num_min_pts_;
  int num_zones_;
  int num_rings_of_interest_;

  double sensor_height_;
  double th_seeds_;
  double th_dist_;
  double th_seeds_v_;
  double th_dist_v_;
  double max_range_;
  double min_range_;
  double uprightness_thr_;
  double adaptive_seed_selection_margin_;
  double min_range_z2_;  // 12.3625
  double min_range_z3_;  // 22.025
  double min_range_z4_;  // 41.35
  double RNR_ver_angle_thr_;
  double RNR_intensity_thr_;

  bool verbose_;
  bool enable_RNR_;
  bool enable_RVPF_;
  bool enable_TGR_;

  int max_flatness_storage_, max_elevation_storage_;
  std::vector<double> update_flatness_[4];
  std::vector<double> update_elevation_[4];

  std::unordered_map<std::string, std::variant<int *, double *, bool *>> variableMap;

  float d_;

  VectorXf normal_;
  MatrixXf pnormal_;
  VectorXf singular_values_;
  Eigen::Matrix3f cov_;
  Eigen::Vector4f pc_mean_;

  // For visualization
  bool visualize_;

  vector<int> num_sectors_each_zone_;
  vector<int> num_rings_each_zone_;

  vector<double> sector_sizes_;
  vector<double> ring_sizes_;
  vector<double> min_ranges_;
  vector<double> elevation_thr_;
  vector<double> flatness_thr_;

  queue<int> noise_idxs_;

  vector<Zone> ConcentricZoneModel_;

  // jsk_recognition_msgs::PolygonArray poly_list_;

  pcl::PointCloud<PointTP> revert_pc_, reject_pc_, noise_pc_, vertical_pc_;
  pcl::PointCloud<PointTP> ground_pc_;

  pcl::PointCloud<pcl::PointXYZINormal> normals_;

  pcl::PointCloud<PointTP> regionwise_ground_, regionwise_nonground_;

  void initialize_zone(Zone &z, int num_sectors, int num_rings);

  void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);
  void flush_patches(std::vector<Zone> &czm);

  void pc2czm(const pcl::PointCloud<PointTP> &src, std::vector<Zone> &czm,
              pcl::PointCloud<PointTP> &cloud_nonground);

  void reflected_noise_removal(pcl::PointCloud<PointTP> &cloud,
                               pcl::PointCloud<PointTP> &cloud_nonground);

  void temporal_ground_revert(pcl::PointCloud<PointTP> &cloud_ground,
                              pcl::PointCloud<PointTP> &cloud_nonground,
                              std::vector<double> ring_flatness,
                              std::vector<RevertCandidate<PointTP>> candidates, int concentric_idx);

  void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

  void update_elevation_thr();
  void update_flatness_thr();

  double xy2theta(const double &x, const double &y);

  double xy2radius(const double &x, const double &y);

  void estimate_plane(const pcl::PointCloud<PointTP> &ground);

  void extract_piecewiseground(const int zone_idx, const pcl::PointCloud<PointTP> &src,
                               pcl::PointCloud<PointTP> &dst,
                               pcl::PointCloud<PointTP> &non_ground_dst);

  void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointTP> &p_sorted,
                             pcl::PointCloud<PointTP> &init_seeds);

  void extract_initial_seeds(const int zone_idx, const pcl::PointCloud<PointTP> &p_sorted,
                             pcl::PointCloud<PointTP> &init_seeds, double th_seed);

  /***
   * For visulization of Ground Likelihood Estimation
   */
  // geometry_msgs::PolygonStamped set_polygons(int zone_idx, int r_idx, int theta_idx, int
  // num_split);
};

template <typename PointTP>
inline void PatchWorkpp<PointTP>::initialize_zone(Zone &z, int num_sectors, int num_rings) {
  z.clear();
  pcl::PointCloud<PointTP> cloud;
  cloud.reserve(1000);
  Ring ring;
  for (int i = 0; i < num_sectors; i++) {
    ring.emplace_back(cloud);
  }
  for (int j = 0; j < num_rings; j++) {
    z.emplace_back(ring);
  }
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::flush_patches_in_zone(Zone &patches, int num_sectors,
                                                        int num_rings) {
  for (int i = 0; i < num_sectors; i++) {
    for (int j = 0; j < num_rings; j++) {
      if (!patches[j][i].points.empty()) patches[j][i].points.clear();
    }
  }
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::flush_patches(vector<Zone> &czm) {
  for (int k = 0; k < num_zones_; k++) {
    for (int i = 0; i < num_rings_each_zone_[k]; i++) {
      for (int j = 0; j < num_sectors_each_zone_[k]; j++) {
        if (!czm[k][i][j].points.empty()) czm[k][i][j].points.clear();
      }
    }
  }

  if (verbose_) cout << "Flushed patches" << endl;
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::estimate_plane(const pcl::PointCloud<PointTP> &ground) {
  pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
  singular_values_ = svd.singularValues();

  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));

  if (normal_(2) < 0) {
    for (int i = 0; i < 3; i++) normal_(i) *= -1;
  }

  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::extract_initial_seeds(const int zone_idx,
                                                        const pcl::PointCloud<PointTP> &p_sorted,
                                                        pcl::PointCloud<PointTP> &init_seeds,
                                                        double th_seed) {
  init_seeds.points.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.points.size(); i++) {
      if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seed) {
      init_seeds.points.push_back(p_sorted.points[i]);
    }
  }
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::extract_initial_seeds(const int zone_idx,
                                                        const pcl::PointCloud<PointTP> &p_sorted,
                                                        pcl::PointCloud<PointTP> &init_seeds) {
  init_seeds.points.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < p_sorted.points.size(); i++) {
      if (p_sorted.points[i].z < adaptive_seed_selection_margin_ * sensor_height_) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (int i = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seeds_) {
      init_seeds.points.push_back(p_sorted.points[i]);
    }
  }
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::reflected_noise_removal(
    pcl::PointCloud<PointTP> &cloud_in, pcl::PointCloud<PointTP> &cloud_nonground) {
  for (int i = 0; i < cloud_in.size(); i++) {
    double r = sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y);
    double z = cloud_in[i].z;
    double ver_angle_in_deg = atan2(z, r) * 180 / M_PI;

    if (ver_angle_in_deg < RNR_ver_angle_thr_ && z < -sensor_height_ - 0.8 &&
        cloud_in[i].custom_field < RNR_intensity_thr_) {
      cloud_nonground.push_back(cloud_in[i]);
      noise_pc_.push_back(cloud_in[i]);
      noise_idxs_.push(i);
    }
  }

  if (verbose_) cout << "[ RNR ] Num of noises : " << noise_pc_.points.size() << endl;
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/

template <typename PointTP>
inline void PatchWorkpp<PointTP>::estimate_ground(
    pcl::PointCloud<PointTP> cloud_in, pcl::PointCloud<PointTP> &cloud_ground,
    pcl::PointCloud<PointTP> &cloud_nonground, double &time_taken_RNR, double &time_taken_CZM,
    double &time_taken_SORT, double &time_taken_GROUND_ESTIMATE, double &time_taken_AGLE,
    double &time_taken_TGR, double &time_taken_UPDATE) {
  unique_lock<recursive_mutex> lock(mutex_);

  time_taken_RNR = 0;
  time_taken_CZM = 0;
  time_taken_SORT = 0;
  time_taken_GROUND_ESTIMATE = 0;
  time_taken_AGLE = 0;
  time_taken_TGR = 0;
  time_taken_UPDATE = 0;

  static double start, t0, t1, t2, end;

  double t_revert = 0.0;
  double t_total_ground = 0.0;
  double t_total_estimate = 0.0;

  start = std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();

  cloud_ground.clear();
  cloud_nonground.clear();

  // 1. Reflected Noise Removal (RNR)
  if (enable_RNR_) reflected_noise_removal(cloud_in, cloud_nonground);

  t1 = std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count();

  // 2. Concentric Zone Model (CZM)
  flush_patches(ConcentricZoneModel_);
  pc2czm(cloud_in, ConcentricZoneModel_, cloud_nonground);

  t2 = std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
           .count();

  int concentric_idx = 0;

  double t_sort = 0;

  std::vector<RevertCandidate<PointTP>> candidates;
  std::vector<double> ringwise_flatness;

  for (int zone_idx = 0; zone_idx < num_zones_; ++zone_idx) {
    auto zone = ConcentricZoneModel_[zone_idx];

    for (int ring_idx = 0; ring_idx < num_rings_each_zone_[zone_idx]; ++ring_idx) {
      for (int sector_idx = 0; sector_idx < num_sectors_each_zone_[zone_idx]; ++sector_idx) {
        if (zone[ring_idx][sector_idx].points.size() < num_min_pts_) {
          cloud_nonground += zone[ring_idx][sector_idx];
          continue;
        }

        // --------- region-wise sorting (faster than global sorting method) ---------------- //
        double t_sort_0 = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();

        sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(),
             point_z_cmp<PointTP>);

        double t_sort_1 = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
        t_sort += (t_sort_1 - t_sort_0);
        // ---------------------------------------------------------------------------------- //

        double t_tmp0 = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();

        // 3. R-VPF + R-GPF
        extract_piecewiseground(zone_idx, zone[ring_idx][sector_idx], regionwise_ground_,
                                regionwise_nonground_);

        double t_tmp1 = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
        t_total_ground += t_tmp1 - t_tmp0;

        double t_tmp2 = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();

        // Status of each patch
        // used in checking uprightness, elevation, and flatness, respectively
        const double ground_uprightness = normal_(2);
        const double ground_elevation = pc_mean_(2, 0);
        const double ground_flatness = singular_values_.minCoeff();
        const double line_variable = singular_values_(1) != 0
                                         ? singular_values_(0) / singular_values_(1)
                                         : std::numeric_limits<double>::max();

        double heading = 0.0;
        for (int i = 0; i < 3; i++) heading += pc_mean_(i, 0) * normal_(i);

        /*
            About 'is_heading_outside' condition, heading should be smaller than 0 theoretically.
            ( Imagine the geometric relationship between the surface normal vector on the ground
           plane and the vector connecting the sensor origin and the mean point of the ground plane
           )

            However, when the patch is far awaw from the sensor origin,
            heading could be larger than 0 even if it's ground due to lack of amount of ground plane
           points.

            Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near
           condition )
        */
        bool is_upright = ground_uprightness > uprightness_thr_;
        bool is_not_elevated = ground_elevation < elevation_thr_[concentric_idx];
        bool is_flat = ground_flatness < flatness_thr_[concentric_idx];
        bool is_near_zone = concentric_idx < num_rings_of_interest_;
        bool is_heading_outside = heading < 0.0;

        /*
            Store the elevation & flatness variables
            for A-GLE (Adaptive Ground Likelihood Estimation)
            and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
        */
        if (is_upright && is_not_elevated && is_near_zone) {
          update_elevation_[concentric_idx].push_back(ground_elevation);
          update_flatness_[concentric_idx].push_back(ground_flatness);

          ringwise_flatness.push_back(ground_flatness);
        }

        // Ground estimation based on conditions
        if (!is_upright) {
          cloud_nonground += regionwise_ground_;
        } else if (!is_near_zone) {
          cloud_ground += regionwise_ground_;
        } else if (!is_heading_outside) {
          cloud_nonground += regionwise_ground_;
        } else if (is_not_elevated || is_flat) {
          cloud_ground += regionwise_ground_;
        } else {
          RevertCandidate<PointTP> candidate(concentric_idx, sector_idx, ground_flatness,
                                             line_variable, pc_mean_, regionwise_ground_);
          candidates.push_back(candidate);
        }
        // Every regionwise_nonground is considered nonground.
        cloud_nonground += regionwise_nonground_;

        double t_tmp3 = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
        t_total_estimate += t_tmp3 - t_tmp2;
      }

      double t_bef_revert = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();

      if (!candidates.empty()) {
        if (enable_TGR_) {
          temporal_ground_revert(cloud_ground, cloud_nonground, ringwise_flatness, candidates,
                                 concentric_idx);
        } else {
          for (size_t i = 0; i < candidates.size(); i++) {
            cloud_nonground += candidates[i].regionwise_ground;
          }
        }

        candidates.clear();
        ringwise_flatness.clear();
      }

      double t_aft_revert = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();

      t_revert += t_aft_revert - t_bef_revert;

      concentric_idx++;
    }
  }

  double t_update = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();

  update_elevation_thr();
  update_flatness_thr();

  end = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();

  time_taken_RNR = t1 - start;
  time_taken_CZM = t2 - t1;
  time_taken_SORT = t_sort;
  time_taken_GROUND_ESTIMATE = t_total_ground;
  time_taken_AGLE = t_total_estimate;
  time_taken_TGR = t_revert;
  time_taken_UPDATE = end - t_update;

  /*
  ofstream pp_time_taken ("pp_time_taken.txt", std::ios_base::app);
  ofstream pp_rnr ("pp_rnr.txt", std::ios_base::app);
  ofstream pp_czm ("pp_czm.txt", std::ios_base::app);
  ofstream pp_sort ("pp_sort.txt", std::ios_base::app);
  ofstream pp_pca ("pp_pca.txt", std::ios_base::app);
  ofstream pp_agle ("pp_agle.txt", std::ios_base::app);
  ofstream pp_tgr ("pp_tgr.txt", std::ios_base::app);
  ofstream pp_update ("pp_update.txt", std::ios_base::app);

  pp_time_taken << time_taken / 1000 << endl;
  pp_rnr << (t1 - start) / 1000  << endl;
  pp_czm << (t2 - t1) / 1000  << endl;
  pp_sort << t_sort / 1000  << endl;
  pp_pca << pca_time_ / 1000  << endl;
  pp_agle << t_total_estimate / 1000  << endl;
  pp_tgr << t_revert / 1000  << endl;
  pp_update << (end - t_update) / 1000  << endl;
  */

  revert_pc_.clear();
  reject_pc_.clear();
  normals_.clear();
  noise_pc_.clear();
  vertical_pc_.clear();
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::update_elevation_thr(void) {
  for (int i = 0; i < num_rings_of_interest_; i++) {
    if (update_elevation_[i].empty()) continue;

    double update_mean = 0.0, update_stdev = 0.0;
    calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
    if (i == 0) {
      elevation_thr_[i] = update_mean + 3 * update_stdev;
      sensor_height_ = -update_mean;
    } else
      elevation_thr_[i] = update_mean + 2 * update_stdev;

    // if (verbose_) cout << "elevation threshold [" << i << "]: " << elevation_thr_[i] << endl;

    int exceed_num = update_elevation_[i].size() - max_elevation_storage_;
    if (exceed_num > 0)
      update_elevation_[i].erase(update_elevation_[i].begin(),
                                 update_elevation_[i].begin() + exceed_num);
  }

  if (verbose_) {
    cout << "sensor height: " << sensor_height_ << endl;
    cout << (boost::format("elevation_thr_  :   %0.4f,  %0.4f,  %0.4f,  %0.4f") %
             elevation_thr_[0] % elevation_thr_[1] % elevation_thr_[2] % elevation_thr_[3])
                .str()
         << endl;
  }

  return;
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::update_flatness_thr(void) {
  for (int i = 0; i < num_rings_of_interest_; i++) {
    if (update_flatness_[i].empty()) break;
    if (update_flatness_[i].size() <= 1) break;

    double update_mean = 0.0, update_stdev = 0.0;
    calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
    flatness_thr_[i] = update_mean + update_stdev;

    // if (verbose_) { cout << "flatness threshold [" << i << "]: " << flatness_thr_[i] << endl; }

    int exceed_num = update_flatness_[i].size() - max_flatness_storage_;
    if (exceed_num > 0)
      update_flatness_[i].erase(update_flatness_[i].begin(),
                                update_flatness_[i].begin() + exceed_num);
  }

  if (verbose_) {
    cout << (boost::format("flatness_thr_   :   %0.4f,  %0.4f,  %0.4f,  %0.4f") % flatness_thr_[0] %
             flatness_thr_[1] % flatness_thr_[2] % flatness_thr_[3])
                .str()
         << endl;
  }

  return;
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::temporal_ground_revert(
    pcl::PointCloud<PointTP> &cloud_ground, pcl::PointCloud<PointTP> &cloud_nonground,
    std::vector<double> ring_flatness, std::vector<RevertCandidate<PointTP>> candidates,
    int concentric_idx) {
  if (verbose_)
    std::cout << "\033[1;34m"
              << "=========== Temporal Ground Revert (TGR) ==========="
              << "\033[0m" << endl;

  double mean_flatness = 0.0, stdev_flatness = 0.0;
  calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

  if (verbose_) {
    cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
         << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness
         << std::endl;
  }

  for (size_t i = 0; i < candidates.size(); i++) {
    RevertCandidate<PointTP> candidate = candidates[i];

    // Debug
    if (verbose_) {
      cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
           << " / flatness: " << candidate.ground_flatness
           << " / line_variable: " << candidate.line_variable
           << " / ground_num : " << candidate.regionwise_ground.size() << "\033[0m" << endl;
    }

    double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
    double prob_flatness =
        1 / (1 + exp((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10)));

    if (candidate.regionwise_ground.size() > 1500 &&
        candidate.ground_flatness < th_dist_ * th_dist_)
      prob_flatness = 1.0;

    double prob_line = 1.0;
    if (candidate.line_variable >
        8.0)  //&& candidate.line_dir > M_PI/4)// candidate.ground_elevation >
              // elevation_thr_[concentric_idx])
    {
      // if (verbose_) cout << "line_dir: " << candidate.line_dir << endl;
      prob_line = 0.0;
    }

    bool revert = prob_line * prob_flatness > 0.5;

    if (concentric_idx < num_rings_of_interest_) {
      if (revert) {
        if (verbose_) {
          cout << "\033[1;32m"
               << "REVERT TRUE"
               << "\033[0m" << endl;
        }

        revert_pc_ += candidate.regionwise_ground;
        cloud_ground += candidate.regionwise_ground;
      } else {
        if (verbose_) {
          cout << "\033[1;31m"
               << "FINAL REJECT"
               << "\033[0m" << endl;
        }
        reject_pc_ += candidate.regionwise_ground;
        cloud_nonground += candidate.regionwise_ground;
      }
    }
  }

  if (verbose_)
    std::cout << "\033[1;34m"
              << "===================================================="
              << "\033[0m" << endl;
}

// For adaptive
template <typename PointTP>
inline void PatchWorkpp<PointTP>::extract_piecewiseground(
    const int zone_idx, const pcl::PointCloud<PointTP> &src, pcl::PointCloud<PointTP> &dst,
    pcl::PointCloud<PointTP> &non_ground_dst) {
  // 0. Initialization
  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!dst.empty()) dst.clear();
  if (!non_ground_dst.empty()) non_ground_dst.clear();

  // 1. Region-wise Vertical Plane Fitting (R-VPF)
  // : removes potential vertical plane under the ground plane
  pcl::PointCloud<PointTP> src_wo_verticals;
  src_wo_verticals = src;

  if (enable_RVPF_) {
    for (int i = 0; i < num_iter_; i++) {
      extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, th_seeds_v_);
      estimate_plane(ground_pc_);

      if (zone_idx == 0 && normal_(2) < uprightness_thr_) {
        pcl::PointCloud<PointTP> src_tmp;
        src_tmp = src_wo_verticals;
        src_wo_verticals.clear();

        Eigen::MatrixXf points(src_tmp.points.size(), 3);
        int j = 0;
        for (auto &p : src_tmp.points) {
          points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;

        for (int r = 0; r < result.rows(); r++) {
          if (result[r] < th_dist_v_ - d_ && result[r] > -th_dist_v_ - d_) {
            non_ground_dst.points.push_back(src_tmp[r]);
            vertical_pc_.points.push_back(src_tmp[r]);
          } else {
            src_wo_verticals.points.push_back(src_tmp[r]);
          }
        }
      } else
        break;
    }
  }

  extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
  estimate_plane(ground_pc_);

  // 2. Region-wise Ground Plane Fitting (R-GPF)
  // : fits the ground plane

  // pointcloud to matrix
  Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
  int j = 0;
  for (auto &p : src_wo_verticals.points) {
    points.row(j++) << p.x, p.y, p.z;
  }

  for (int i = 0; i < num_iter_; i++) {
    ground_pc_.clear();

    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (i < num_iter_ - 1) {
        if (result[r] < th_dist_ - d_) {
          ground_pc_.points.push_back(src_wo_verticals[r]);
        }
      } else {  // Final stage
        if (result[r] < th_dist_ - d_) {
          dst.points.push_back(src_wo_verticals[r]);
        } else {
          non_ground_dst.points.push_back(src_wo_verticals[r]);
        }
      }
    }

    if (i < num_iter_ - 1)
      estimate_plane(ground_pc_);
    else
      estimate_plane(dst);
  }
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::calc_mean_stdev(std::vector<double> vec, double &mean,
                                                  double &stdev) {
  if (vec.size() <= 1) return;

  mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

  for (int i = 0; i < vec.size(); i++) {
    stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
  }
  stdev /= vec.size() - 1;
  stdev = sqrt(stdev);
}

template <typename PointTP>
inline double PatchWorkpp<PointTP>::xy2theta(const double &x, const double &y) {  // 0 ~ 2 * PI
  // if (y >= 0) {
  //     return atan2(y, x); // 1, 2 quadrant
  // } else {
  //     return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
  // }

  double angle = atan2(y, x);
  return angle > 0 ? angle : 2 * M_PI + angle;
}

template <typename PointTP>
inline double PatchWorkpp<PointTP>::xy2radius(const double &x, const double &y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

template <typename PointTP>
inline void PatchWorkpp<PointTP>::pc2czm(const pcl::PointCloud<PointTP> &src,
                                         std::vector<Zone> &czm,
                                         pcl::PointCloud<PointTP> &cloud_nonground) {
  for (int i = 0; i < src.size(); i++) {
    if ((!noise_idxs_.empty()) && (i == noise_idxs_.front())) {
      noise_idxs_.pop();
      continue;
    }

    PointTP pt = src.points[i];

    double r = xy2radius(pt.x, pt.y);
    if ((r <= max_range_) && (r > min_range_)) {
      double theta = xy2theta(pt.x, pt.y);

      int zone_idx = 0;
      if (r < min_ranges_[1])
        zone_idx = 0;
      else if (r < min_ranges_[2])
        zone_idx = 1;
      else if (r < min_ranges_[3])
        zone_idx = 2;
      else
        zone_idx = 3;

      int ring_idx = min(static_cast<int>(((r - min_ranges_[zone_idx]) / ring_sizes_[zone_idx])),
                         num_rings_each_zone_[zone_idx] - 1);
      int sector_idx = min(static_cast<int>((theta / sector_sizes_[zone_idx])),
                           num_sectors_each_zone_[zone_idx] - 1);

      czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
    } else {
      cloud_nonground.push_back(pt);
    }
  }

  if (verbose_) cout << "[ CZM ] Divides pointcloud into the concentric zone model" << endl;
}

#endif
