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

#ifndef ALIB_METRICS_H
#define ALIB_METRICS_H

#define GROUND_LABELS 0
#define GROUND_LABELS_PLUS_TERRAIN 1
#define GROUND_LABELS_PLUS_TERRAIN_VEGETATION 2
#define GROUND_LABELS_TRANSVERSABLE 3

#include <iostream>
#include <mutex>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"

struct AlfaBasicMetrics {
  int number_input_points;
  int number_output_points;
};

struct AlfaGroundSegmentationMetrics {
  int number_input_ground_points;
  int number_output_ground_points;
  float true_positive;
  float false_positive;
  float false_negative;
  float true_negative;
  float true_positive_rate;
  float true_negative_rate;
  float false_positive_rate;
  float false_negative_rate;
  float positive_predictive_value;  //
  float negative_predictive_value;  //
  float f1_score;
  float accuracy;
  float IoUg;
  AlfaBasicMetrics basic_metrics;
};

AlfaGroundSegmentationMetrics evaluate_ground_segmentation_method(
    AlfaNode* node, int map_type);
#endif  // ALIB_METRICS_H