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

#ifndef ALIB_OCTREE_H
#define ALIB_OCTREE_H

#include <iostream>
#include <mutex>
#include <vector>

#include "alfa_node.hpp"

struct AlfaBB {
  float min_x;
  float min_y;
  float min_z;
  float max_x;
  float max_y;
  float max_z;
  AlfaBB(float min_x = 0, float min_y = 0, float min_z = 0, float max_x = 0, float max_y = 0,
         float max_z = 0)
      : min_x(min_x), min_y(min_y), min_z(min_z), max_x(max_x), max_y(max_y), max_z(max_z) {}
};

struct AlfaOctreeNode {
  unsigned int index;             // 4 bytes
  unsigned int depth;             // 4 bytes
  AlfaOctreeNode *parent;         // 8 bytes
  AlfaOctreeNode *branchs[8];     // 8 * 8 bytes
  unsigned char occupation_code;  // 1 byte
  AlfaBB bounding_box;            // 6 * 4 bytes
};

class AlfaOctree {
 public:
  AlfaOctree(AlfaBB, int, bool multithread);
  ~AlfaOctree();

  void insert_point(AlfaPoint point);
  void insert_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);

  int get_number_of_points();
  int get_number_of_nodes();
  int get_number_of_discarded_points();
  float get_resolution_z();
  float get_resolution_x();
  float get_resolution_y();
  std::vector<unsigned char> get_occupation_code_DFS();
  std::vector<AlfaPoint> convert_to_pointcloud();
  void convert_to_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);
  void init_octree_from_occupation_code_DFS(std::vector<unsigned char> DFS_code,
                                            AlfaBB bounding_box);

  void reset_octree(AlfaBB);
  void reset_octree();

 private:
  int number_of_points;
  int number_of_discarded_points;
  int number_of_nodes;
  bool multithread;
  AlfaOctreeNode *root;
  AlfaOctreeNode *last_inserted_node[8];
  unsigned int max_depth;
  float *voxel_sizes_x;
  float *voxel_sizes_y;
  float *voxel_sizes_z;
  std::mutex convert_to_pointcloud_mutex;
  std::mutex number_of_nodes_mutex;

  void insert_point_root_branch(AlfaPoint point, int childIndex);
  void insert_pointcloud_multi_thread_vector_handler(std::vector<AlfaPoint> pointcloud,
                                                     int childIndex);
  void insert_pointcloud_multi_thread_handler(pcl::PointCloud<AlfaPoint>::Ptr pointcloud,
                                              int childIndex);
  void init_node_from_occupation_code_DFS(AlfaOctreeNode *node, std::vector<unsigned char> &code,
                                          unsigned int *index);

  void insert_pointcloud_multi_thread(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);

  void insert_pointcloud_single_thread(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);

  std::vector<unsigned char> get_occupation_code_DFS_multi_thread();
  std::vector<unsigned char> get_occupation_code_DFS_single_thread();

  std::vector<AlfaPoint> convert_to_pointcloud_multi_thread();
  std::vector<AlfaPoint> convert_to_pointcloud_single_thread();

  void delete_node(AlfaOctreeNode *node);
  bool is_leaf(AlfaOctreeNode *node);
  bool is_point_inside_BB(AlfaPoint, AlfaBB);
  void set_node_BB(AlfaOctreeNode *node, int childIndex);
  void retrieve_occupation_code_DFS(AlfaOctreeNode *node, std::vector<unsigned char> &code);
  void retrieve_points(AlfaOctreeNode *node, std::vector<AlfaPoint> &points);
  bool setup_thread(std::thread &thread, int core_id);
  AlfaPoint get_centroid(AlfaOctreeNode *node);
};

#endif  // ALIB_OCTREE_H
