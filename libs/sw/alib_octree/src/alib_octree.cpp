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

#include "alib_octree.hpp"

#include <pthread.h>
#include <sched.h>

#include <thread>

AlfaOctree::AlfaOctree(AlfaBB bounding_box, int max_depth = 20, bool multithread = false) {
  this->number_of_points = 0;
  this->number_of_nodes = 1;
  this->number_of_discarded_points = 0;
  this->max_depth = max_depth;
  this->root = new AlfaOctreeNode();
  this->root->index = 0;
  this->root->depth = 0;
  this->root->occupation_code = 0;
  this->root->parent = NULL;
  this->root->bounding_box = bounding_box;
  this->multithread = multithread;

  this->voxel_sizes_x = new float[max_depth + 1];
  this->voxel_sizes_y = new float[max_depth + 1];
  this->voxel_sizes_z = new float[max_depth + 1];

  for (int i = 0; i < max_depth + 1; i++) {
    this->voxel_sizes_x[i] = (bounding_box.max_x - bounding_box.min_x) / (1 << i);
    this->voxel_sizes_y[i] = (bounding_box.max_y - bounding_box.min_y) / (1 << i);
    this->voxel_sizes_z[i] = (bounding_box.max_z - bounding_box.min_z) / (1 << i);
  }

  for (int i = 0; i < 8; i++) {
    this->root->branchs[i] = NULL;
    this->last_inserted_node[i] = this->root->branchs[i];
  }
}

AlfaOctree::~AlfaOctree() {
  reset_octree();
  delete this->root;
}

void AlfaOctree::insert_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (this->multithread) {
    insert_pointcloud_multi_thread(pointcloud);
  } else {
    insert_pointcloud_single_thread(pointcloud);
  }
}

void AlfaOctree::insert_pointcloud_multi_thread(pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  for (int i = 0; i < 8; i++) {
    if (this->root->branchs[i] == NULL) {
      this->root->branchs[i] = new AlfaOctreeNode();
      this->root->branchs[i]->index = this->number_of_nodes++;
      this->root->branchs[i]->depth = this->root->depth + 1;
      this->root->branchs[i]->occupation_code = 0;
      this->root->branchs[i]->parent = this->root;
      for (int j = 0; j < 8; j++) {
        this->root->branchs[i]->branchs[j] = NULL;
      }
      set_node_BB(this->root->branchs[i], i);
      this->last_inserted_node[i] = this->root->branchs[i];
    }
  }

  std::thread threads[8];

  vector<AlfaPoint> pointclouds[8];

  for (const auto &point : *pointcloud) {
    for (int j = 0; j < 8; j++) {
      if (is_point_inside_BB(point, this->root->branchs[j]->bounding_box)) {
        pointclouds[j].push_back(point);
        break;
      }
    }
  }

  // Create and run threads
  for (int i = 0; i < 8; i++) {
    threads[i] = std::thread(&AlfaOctree::insert_pointcloud_multi_thread_vector_handler, this,
                             pointclouds[i], i);

    // Set thread affinity
    setup_thread(threads[i], i);
  }

  // Wait for all threads to finish
  for (int i = 0; i < 8; i++) {
    if (threads[i].joinable()) {
      threads[i].join();
    }
  }

  // Set the right occupation code in root
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if (this->root->branchs[i]->branchs[j] != NULL) {
        this->root->occupation_code |= (1 << i);
        break;
      }
    }
  }
}

void AlfaOctree::insert_pointcloud_single_thread(pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  for (const auto &point : *pointcloud) insert_point(point);
}

void AlfaOctree::insert_pointcloud_multi_thread_vector_handler(vector<AlfaPoint> pointcloud,
                                                               int childIndex) {
  for (size_t i = 0; i < pointcloud.size(); i++) {
    insert_point_root_branch(pointcloud[i], childIndex);
  }
}

void AlfaOctree::insert_point(AlfaPoint point) {
  if (!is_point_inside_BB(point, root->bounding_box)) {
    this->number_of_discarded_points++;
    return;
  }
  this->number_of_points++;

  // Get centroid of the node
  AlfaPoint centroid = get_centroid(this->root);
  // Determine the position of the point within the octree
  int indexX = 0;
  int indexY = 0;
  int indexZ = 0;

  if (point.x >= centroid.x) indexX = 1;
  if (point.y >= centroid.y) indexY = 1;
  if (point.z >= centroid.z) indexZ = 1;

  // cout << "norm" << normX << " " << normY << " " << normZ << endl;
  // cout << "index: " << indexX << " " << indexY << " " << indexZ << endl;

  // Calculate the index of the child octant the point falls into
  int childIndex = (indexZ << 2) | (indexY << 1) | indexX;

  // Set the corresponding bit in the occupancy code for the current depth
  this->root->occupation_code |= (1 << childIndex);

  if (this->root->branchs[childIndex] == NULL) {
    this->root->branchs[childIndex] = new AlfaOctreeNode();
    this->root->branchs[childIndex]->index = this->number_of_nodes++;
    this->root->branchs[childIndex]->depth = this->root->depth + 1;
    this->root->branchs[childIndex]->occupation_code = 0;
    this->root->branchs[childIndex]->parent = this->root;
    for (int i = 0; i < 8; i++) {
      this->root->branchs[childIndex]->branchs[i] = NULL;
    }
    set_node_BB(this->root->branchs[childIndex], childIndex);
    this->last_inserted_node[childIndex] = this->root->branchs[childIndex];
  }

  insert_point_root_branch(point, childIndex);
}

void AlfaOctree::insert_point_root_branch(AlfaPoint point, int rootchildIndex) {
  AlfaOctreeNode *pointer = this->last_inserted_node[rootchildIndex];

  // Go up the tree until the point is inside the bounding box
  while (!is_point_inside_BB(point, pointer->bounding_box)) {
    if (pointer == this->root->branchs[rootchildIndex]) {
      return;
    }
    pointer = pointer->parent;
  }
  // Go down the tree until the point is inside a leaf node
  do {
    // Get centroid of the node
    AlfaPoint centroid = get_centroid(pointer);
    // Determine the position of the point within the octree
    int indexX = 0;
    int indexY = 0;
    int indexZ = 0;

    if (point.x >= centroid.x) indexX = 1;
    if (point.y >= centroid.y) indexY = 1;
    if (point.z >= centroid.z) indexZ = 1;

    // cout << "norm" << normX << " " << normY << " " << normZ << endl;
    // cout << "index: " << indexX << " " << indexY << " " << indexZ << endl;

    // Calculate the index of the child octant the point falls into
    int childIndex = (indexZ << 2) | (indexY << 1) | indexX;

    // Set the corresponding bit in the occupancy code for the current depth
    pointer->occupation_code |= (1 << childIndex);

    if (!is_leaf(pointer)) {
      if (pointer->branchs[childIndex] == NULL) {
        pointer->branchs[childIndex] = new AlfaOctreeNode();
        number_of_nodes_mutex.lock();
        pointer->branchs[childIndex]->index = this->number_of_nodes++;
        number_of_nodes_mutex.unlock();
        pointer->branchs[childIndex]->depth = pointer->depth + 1;
        pointer->branchs[childIndex]->occupation_code = 0;
        pointer->branchs[childIndex]->parent = pointer;
        for (int i = 0; i < 8; i++) {
          pointer->branchs[childIndex]->branchs[i] = NULL;
        }
        set_node_BB(pointer->branchs[childIndex], childIndex);
      } else {
        pointer = pointer->branchs[childIndex];
      }
    } else
      break;
  } while (!is_leaf(pointer));

  this->last_inserted_node[rootchildIndex] = pointer;
}

void AlfaOctree::retrieve_occupation_code_DFS(AlfaOctreeNode *node, vector<unsigned char> &code) {
  if (!is_leaf(node)) {
    code.push_back(node->occupation_code);
    for (int i = 0; i < 8; i++) {
      if (node->branchs[i] != NULL) {
        retrieve_occupation_code_DFS(node->branchs[i], code);
      }
    }
  }
}

vector<unsigned char> AlfaOctree::get_occupation_code_DFS_multi_thread() {
  vector<unsigned char> code[8];
  vector<unsigned char> final_code;
  std::thread threads[8];
  for (int i = 0; i < 8; i++) {
    if (this->root->branchs[i] != NULL) {
      threads[i] = std::thread(&AlfaOctree::retrieve_occupation_code_DFS, this,
                               this->root->branchs[i], std::ref(code[i]));

      // Set thread affinity
      setup_thread(threads[i], i);
    }
  }

  for (int i = 0; i < 8; i++) {
    if (threads[i].joinable()) {
      // wait for the thread to finish
      threads[i].join();
    }
  }

  final_code.push_back(this->root->occupation_code);
  for (int i = 0; i < 8; i++) {
    if (code[i].size() > 0) final_code.insert(final_code.end(), code[i].begin(), code[i].end());
  }
  return final_code;
}

vector<unsigned char> AlfaOctree::get_occupation_code_DFS_single_thread() {
  vector<unsigned char> final_code;

  retrieve_occupation_code_DFS(this->root, final_code);
  return final_code;
}

vector<unsigned char> AlfaOctree::get_occupation_code_DFS() {
  if (this->multithread) {
    return get_occupation_code_DFS_multi_thread();
  } else {
    return get_occupation_code_DFS_single_thread();
  }
}

AlfaPoint AlfaOctree::get_centroid(AlfaOctreeNode *node) {
  AlfaPoint centroid;
  centroid.x = (node->bounding_box.min_x + node->bounding_box.max_x) / 2;
  centroid.y = (node->bounding_box.min_y + node->bounding_box.max_y) / 2;
  centroid.z = (node->bounding_box.min_z + node->bounding_box.max_z) / 2;

  return centroid;
}

void AlfaOctree::retrieve_points(AlfaOctreeNode *node, vector<AlfaPoint> &points) {
  if (is_leaf(node)) {
    AlfaPoint centroid = get_centroid(node);
    convert_to_pointcloud_mutex.lock();
    points.push_back(centroid);
    convert_to_pointcloud_mutex.unlock();
  } else {
    for (int i = 0; i < 8; i++) {
      if (node->branchs[i] != NULL) {
        retrieve_points(node->branchs[i], points);
      }
    }
  }
}

vector<AlfaPoint> AlfaOctree::convert_to_pointcloud() {
  if (this->multithread) {
    return convert_to_pointcloud_multi_thread();
  } else {
    return convert_to_pointcloud_single_thread();
  }
}

vector<AlfaPoint> AlfaOctree::convert_to_pointcloud_multi_thread() {
  vector<AlfaPoint> points;
  std::thread threads[8];
  for (int i = 0; i < 8; i++) {
    if (this->root->branchs[i] != NULL) {
      threads[i] =
          std::thread(&AlfaOctree::retrieve_points, this, this->root->branchs[i], std::ref(points));
      // Set thread affinity
      setup_thread(threads[i], i);
    }
  }

  for (int i = 0; i < 8; i++) {
    if (this->root->branchs[i] != NULL) {
      threads[i].join();
    }
  }
  return points;
}

vector<AlfaPoint> AlfaOctree::convert_to_pointcloud_single_thread() {
  vector<AlfaPoint> points;
  retrieve_points(this->root, points);

  return points;
}

float AlfaOctree::get_resolution_z() { return this->voxel_sizes_z[max_depth]; }

float AlfaOctree::get_resolution_x() { return this->voxel_sizes_x[max_depth]; }

float AlfaOctree::get_resolution_y() { return this->voxel_sizes_y[max_depth]; }

void AlfaOctree::init_octree_from_occupation_code_DFS(vector<unsigned char> code,
                                                      AlfaBB bounding_box) {
  this->reset_octree(bounding_box);
  unsigned int index = 0;
  init_node_from_occupation_code_DFS(this->root, code, &index);
}

void AlfaOctree::init_node_from_occupation_code_DFS(AlfaOctreeNode *node,
                                                    vector<unsigned char> &code,
                                                    unsigned int *index) {
  if (*index < code.size() && !is_leaf(node)) {
    node->occupation_code = code[(*index)++];
    for (int i = 0; i < 8; i++) {
      if (node->occupation_code & (1 << i)) {
        node->branchs[i] = new AlfaOctreeNode();
        node->branchs[i]->index = this->number_of_nodes++;
        node->branchs[i]->depth = node->depth + 1;
        node->branchs[i]->parent = node;
        set_node_BB(node->branchs[i], i);
        init_node_from_occupation_code_DFS(node->branchs[i], code, index);
      } else
        node->branchs[i] = NULL;
    }
  }
}

bool AlfaOctree::setup_thread(std::thread &thread, int core_id) {
  // Set thread affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);  // Pin thread to core_id

  int rc = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
  if (rc != 0) {
    return false;
  }

  return true;
}

bool AlfaOctree::is_leaf(AlfaOctreeNode *node) {
  if (node->depth >= this->max_depth)
    return true;
  else
    return false;
}

int AlfaOctree::get_number_of_points() { return this->number_of_points; }

int AlfaOctree::get_number_of_nodes() { return this->number_of_nodes; }

int AlfaOctree::get_number_of_discarded_points() { return this->number_of_discarded_points; }

bool AlfaOctree::is_point_inside_BB(AlfaPoint point, AlfaBB bounding_box) {
  if (point.x >= bounding_box.min_x && point.x <= bounding_box.max_x &&
      point.y >= bounding_box.min_y && point.y <= bounding_box.max_y &&
      point.z >= bounding_box.min_z && point.z <= bounding_box.max_z)
    return true;
  else
    return false;
}

void AlfaOctree::set_node_BB(AlfaOctreeNode *node, int childIndex) {
  node->bounding_box.min_x =
      node->parent->bounding_box.min_x + (childIndex & 1) * voxel_sizes_x[node->depth];

  node->bounding_box.max_x = node->bounding_box.min_x + voxel_sizes_x[node->depth];

  node->bounding_box.min_y =
      node->parent->bounding_box.min_y + ((childIndex >> 1) & 1) * voxel_sizes_y[node->depth];

  node->bounding_box.max_y = node->bounding_box.min_y + voxel_sizes_y[node->depth];

  node->bounding_box.min_z =
      node->parent->bounding_box.min_z + ((childIndex >> 2) & 1) * voxel_sizes_z[node->depth];

  node->bounding_box.max_z = node->bounding_box.min_z + voxel_sizes_z[node->depth];
}

void AlfaOctree::reset_octree(AlfaBB bounding_box) {
  this->root->bounding_box = bounding_box;
  reset_octree();
}

void AlfaOctree::reset_octree() {
  for (int i = 0; i < 8; i++) {
    if (this->root->branchs[i] != NULL) delete_node(this->root->branchs[i]);
  }
  this->number_of_points = 0;
  this->number_of_discarded_points = 0;
  this->number_of_nodes = 1;
}

void AlfaOctree::delete_node(AlfaOctreeNode *node) {
  for (int i = 0; i < 8; i++) {
    if (node->branchs[i] != NULL) {
      delete_node(node->branchs[i]);
      delete node->branchs[i];
      this->number_of_nodes--;
      node->branchs[i] = NULL;
    }
  }
}
