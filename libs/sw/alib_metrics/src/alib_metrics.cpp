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

#include "alib_metrics.hpp"

const std::vector<int> alfa_label_to_alfa_ground = {
    ALFA_LABEL_ROAD, ALFA_LABEL_PARKING, ALFA_LABEL_SIDEWALK,
    ALFA_LABEL_OTHER_GROUND, ALFA_LABEL_LANE_MARKING};

const std::vector<int> alfa_label_to_alfa_ground_plus_terrain = {
    ALFA_LABEL_ROAD,         ALFA_LABEL_PARKING, ALFA_LABEL_SIDEWALK,
    ALFA_LABEL_OTHER_GROUND, ALFA_LABEL_TERRAIN, ALFA_LABEL_LANE_MARKING};

const std::vector<int> alfa_label_to_alfa_ground_plus_terrain_vegetation = {
    ALFA_LABEL_ROAD,         ALFA_LABEL_PARKING, ALFA_LABEL_SIDEWALK,
    ALFA_LABEL_OTHER_GROUND, ALFA_LABEL_TERRAIN, ALFA_LABEL_VEGETATION,
    ALFA_LABEL_LANE_MARKING};

const std::vector<int> alfa_label_transversable_to_alfa_ground = {
    ALFA_LABEL_ROAD, ALFA_LABEL_PARKING, ALFA_LABEL_OTHER_GROUND,
    ALFA_LABEL_LANE_MARKING};

bool is_label_in_vector(const std::vector<int>& labels, int label) {
  return std::find(labels.begin(), labels.end(), label) != labels.end();
}

AlfaGroundSegmentationMetrics evaluate_ground_segmentation_method(
    AlfaNode* node, int map_type = 0) {
  AlfaGroundSegmentationMetrics metrics;
  metrics.number_input_ground_points = 0;
  metrics.number_output_ground_points = 0;
  metrics.true_positive = 0;
  metrics.false_positive = 0;
  metrics.false_negative = 0;
  metrics.true_negative = 0;
  metrics.basic_metrics.number_input_points = node->get_input_pointcloud_size();
  metrics.basic_metrics.number_output_points =
      node->get_output_pointcloud_size();
  metrics.true_positive_rate = 0;
  metrics.true_negative_rate = 0;
  metrics.false_positive_rate = 0;
  metrics.false_negative_rate = 0;
  metrics.positive_predictive_value = 0;
  metrics.negative_predictive_value = 0;
  metrics.f1_score = 0;
  metrics.accuracy = 0;
  metrics.IoUg = 0;

  for (std::uint32_t i = 0; i < node->get_input_pointcloud_size(); i++) {
    AlfaPoint input_point = node->get_point_input_pointcloud(i);
    AlfaPoint output_point = node->get_point_output_pointcloud(i);

    bool label_found = false;
    int input_label = input_point.custom_field & 0xFF;
    int output_label = output_point.custom_field & 0xFF;

    switch (map_type) {
      case GROUND_LABELS:
        label_found =
            is_label_in_vector(alfa_label_to_alfa_ground, input_label);
        break;
      case GROUND_LABELS_PLUS_TERRAIN:
        label_found = is_label_in_vector(alfa_label_to_alfa_ground_plus_terrain,
                                         input_label);
        break;
      case GROUND_LABELS_PLUS_TERRAIN_VEGETATION:
        label_found = is_label_in_vector(
            alfa_label_to_alfa_ground_plus_terrain_vegetation, input_label);
        break;
      case GROUND_LABELS_TRANSVERSABLE:
        label_found = is_label_in_vector(
            alfa_label_transversable_to_alfa_ground, input_label);
        break;
      default:
        throw std::invalid_argument("Unknown map type");
    }

    if (label_found) {
      metrics.number_input_ground_points++;
      if (output_label == ALFA_LABEL_GROUND) {
        metrics.number_output_ground_points++;
        metrics.true_positive++;
      } else {
        metrics.false_negative++;
      }
    } else {
      if (output_label == ALFA_LABEL_GROUND) {
        metrics.number_output_ground_points++;
        metrics.false_positive++;
      } else {
        metrics.true_negative++;
      }
    }
  }

  metrics.true_positive_rate =
      metrics.true_positive / (metrics.true_positive + metrics.false_negative);
  metrics.true_negative_rate =
      metrics.true_negative / (metrics.true_negative + metrics.false_positive);
  metrics.false_positive_rate =
      metrics.false_positive / (metrics.false_positive + metrics.true_negative);
  metrics.false_negative_rate =
      metrics.false_negative / (metrics.false_negative + metrics.true_positive);
  metrics.positive_predictive_value =
      metrics.true_positive / (metrics.true_positive + metrics.false_positive);
  metrics.negative_predictive_value =
      metrics.true_negative / (metrics.true_negative + metrics.false_negative);
  metrics.f1_score =
      2 * (metrics.true_positive_rate * metrics.positive_predictive_value) /
      (metrics.true_positive_rate + metrics.positive_predictive_value);
  metrics.accuracy = (metrics.true_positive + metrics.true_negative) /
                     (metrics.true_positive + metrics.true_negative +
                      metrics.false_positive + metrics.false_negative);
  metrics.IoUg =
      metrics.true_positive /
      (metrics.true_positive + metrics.false_positive + metrics.false_negative);

  return metrics;
}