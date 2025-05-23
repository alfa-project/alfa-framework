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

#include <flann/util/matrix.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <time.h>

#include <cmath>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "rclcpp/rclcpp.hpp"

template class pcl::KdTreeFLANN<AlfaPoint>;

// Node parameters
#define NODE_NAME "ext_dior"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0

// Dior definitions
#define INTENSITY_PARAM_ID 0           // intensity thr
#define DISTANCE_TH_PARAM_ID 1         // radius multiplier
#define SEARCH_RADIUS_MIN_PARAM_ID 2   // min search radius
#define NUMBER_OF_NEIGH_TH_PARAM_ID 3  // neighbou thr
#define ANGLE_RES_PARAM_ID 4           // angular res
#define NUMBERTHREADS_ID 5

// DIOR parameters
struct ProcessingParams {
  float angle_resolution;
  float search_radius_min;
  float distance_th;
  float intensity_th;
  int number_of_neigh_th;
};

/**
 * This function performs a dynamic radius search for a given point using a
 * KdTree search algorithm.
 *
 * @param point The point for which the search is performed
 * @param kdtree The KdTree data structure that is used for the search
 * @param angle_resolution The angle resolution used for the search
 * @param search_radius_min The minimum search radius used for the search
 * @param distance_th The distance threshold used for the search
 * @param number_of_neigh_th The number of neighbors threshold used for the
 * search
 *
 * @return Returns true if the number of neighbors found is greater than or
 * equal to the specified threshold, false otherwise
 */

bool dynamicRadiusSearch(const AlfaPoint& point,
                         pcl::KdTreeFLANN<AlfaPoint>& kdtree,
                         const ProcessingParams& params) {
  float distance_squared =
      point.x * point.x + point.y * point.y + point.z * point.z;
  float search_radius;

  if (distance_squared < params.distance_th * params.distance_th) {
    search_radius = params.search_radius_min;
  } else {
    float distance = std::sqrt(distance_squared);
    search_radius =
        params.search_radius_min * (distance * params.angle_resolution);
  }

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  int neighbors = kdtree.radiusSearch(
      point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  return neighbors >= params.number_of_neigh_th;
}

/**
 * @brief Performs processing on a sequence of points using KD-tree filtering.
 *
 * This function evaluates each point in the input point cloud based on
 * intensity and dynamic radius neighbor conditions. Points that pass the
 * filters are added to the output cloud. Radius search uses a KD-tree and
 * adapts based on distance and angular resolution parameters.
 *
 * @param input_pc Pointer to the input point cloud.
 * @param output_pc Pointer to the output point cloud (will be filled by this
 * function).
 * @param kdtree Reference to the KD-tree used for neighbor search.
 * @param params Struct containing all the filtering parameters.
 */
void worker(pcl::PointCloud<AlfaPoint>::Ptr input_pc,
            pcl::PointCloud<AlfaPoint>::Ptr output_pc,
            pcl::KdTreeFLANN<AlfaPoint>& kdtree,
            const ProcessingParams& params) {
  // Pre-allocate space in the output cloud
  output_pc->points.reserve(input_pc->points.size());

  for (const auto& point : input_pc->points) {
    // First step: check if the point's intensity is above the threshold
    if (point.custom_field > params.intensity_th) {
      output_pc->points.push_back(point);
    }
    // If not, apply dynamic radius search to determine if it should be kept
    else if (dynamicRadiusSearch(point, kdtree, params)) {
      output_pc->points.push_back(point);
    }
  }

  output_pc->width = static_cast<uint32_t>(output_pc->points.size());
  output_pc->height = 1;
  output_pc->is_dense = true;
}

/**
 * @brief Handler function that filters (Using DIOR) the point cloud retrieved from DEFAULT_TOPIC
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode* node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN);
#else
  // Create the KDTree
  pcl::KdTreeFLANN<AlfaPoint> kdtree;
  kdtree.setInputCloud(node->get_input_pointcloud());

  // Parameters
  ProcessingParams params;
  params.angle_resolution = node->get_extension_parameter("angular_res");
  params.search_radius_min = node->get_extension_parameter("min_search_radius");
  params.distance_th = node->get_extension_parameter("distance_thr");
  params.intensity_th = node->get_extension_parameter("intensity_thr");
  params.number_of_neigh_th =
      node->get_extension_parameter("num_neighbors_thr");

  // Thread-related setup
  int thread_count = node->get_extension_parameter("num_threads");
  std::vector<std::thread> thread_list;
  std::vector<pcl::PointCloud<AlfaPoint>::Ptr> output_pc(thread_count);

  const auto input = node->get_input_pointcloud();
  const size_t total_points = input->points.size();
  const size_t points_per_thread = total_points / thread_count;

  for (int i = 0; i < thread_count; ++i) {
    size_t start_idx = i * points_per_thread;
    size_t end_idx =
        (i == thread_count - 1) ? total_points : start_idx + points_per_thread;

    pcl::PointCloud<AlfaPoint>::Ptr input_slice(new pcl::PointCloud<AlfaPoint>);
    pcl::PointCloud<AlfaPoint>::Ptr output_slice(
        new pcl::PointCloud<AlfaPoint>);
    output_pc[i] = output_slice;

    // Slice the input
    input_slice->points.insert(input_slice->points.end(),
                               input->points.begin() + start_idx,
                               input->points.begin() + end_idx);
    input_slice->width = static_cast<uint32_t>(input_slice->points.size());
    input_slice->height = 1;
    input_slice->is_dense = input->is_dense;

    // Launch thread
    thread_list.emplace_back(worker, input_slice, output_slice,
                             std::ref(kdtree), params);
  }

  // Wait for threads
  for (auto& t : thread_list) t.join();

  // Merge outputs
  auto final_output = node->get_output_pointcloud();
  final_output->points.reserve(total_points);
  for (const auto& pc : output_pc) *final_output += *pc;
#endif
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point
 * cloud.
 */
void post_processing(AlfaNode* node) {
#ifdef EXT_HARDWARE
// HARDWARE extension ToDo
#endif

  node->publish_pointcloud();
}

/**
 * @brief Entry point of the program.
 *
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Get subscriber topic from command line arguments
  std::string subscriber_topic = DEFAULT_TOPIC;
  if (argc > 1) {
    subscriber_topic = argv[1];
  }

#ifdef EXT_HARDWARE
  AlfaHardwareSupport hardware_support{false, true};
#else
  AlfaHardwareSupport hardware_support{false, false};
#endif

  AlfaConfiguration conf;

  // Prepare the DIOR default configuration
  AlfaExtensionParameter parameters[10];

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 1;
  conf.metrics_publishing_type = ALL_METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_INTENSITY;

  parameters[0].parameter_value = 4.0;
  parameters[0].parameter_name = "intensity_thr";
  parameters[1].parameter_value = 0.9;
  parameters[1].parameter_name = "distance_thr";
  parameters[2].parameter_value = 0.1;
  parameters[2].parameter_name = "min_search_radius";
  parameters[3].parameter_value = 20.0;
  parameters[3].parameter_name = "num_neighbors_thr";
  parameters[4].parameter_value = 0.3;
  parameters[4].parameter_name = "angular_res";
  parameters[5].parameter_value = 1;
  parameters[5].parameter_name = "num_threads";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(
      std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;

  return 0;
}
