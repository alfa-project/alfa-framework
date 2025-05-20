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

#include "alfa_node.hpp"

using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;


// Method for publishing a AlfaMetrics object
void AlfaNode::publish_metrics(alfa_msg::msg::AlfaMetrics &metrics) {
#ifdef ALFA_VERBOSE
  verbose_info("publish_metrics", "publishing metrics");
#endif
  metrics_publisher->publish(metrics);  // publish the metrics
}

void AlfaNode::metrics_update() {
  auto duration_full_processing = duration_cast<microseconds>(
      full_processing_metric.stop - full_processing_metric.start);
  auto duration_handler =
      duration_cast<microseconds>(handler_metric.stop - handler_metric.start);
  auto duration_publishing = duration_cast<microseconds>(
      publishing_metric.stop - publishing_metric.start);

  /*FIXME*/ if (duration_handler.count() < 200000)
    handler_metric.message.metric = duration_handler.count();
  publishing_metric.message.metric = duration_publishing.count();
  full_processing_metric.message.metric =
      duration_full_processing.count() + duration_publishing.count();
  number_of_processed_points.message.metric = input_pointcloud->size();
}

void AlfaNode::metrics_setup() {
  // Metrics
  handler_metric.message.units = "us";
  handler_metric.message.metric_name = "Handler processing time";
  handler_metric.message.metric = 0;

  full_processing_metric.message.units = "us";
  full_processing_metric.message.metric_name = "Full processing time";
  full_processing_metric.message.metric = 0;

  number_of_processed_points.message.units = "points";
  number_of_processed_points.message.metric_name = "Processed Points";

  for (int i = 0; i < 20; i++) {
    debug_points_message[i].units = "";
    debug_points_message[i].metric_name = "Debug point " + std::to_string(i);
    debug_points_message[i].metric = 0;
  }
}

void AlfaNode::metrics_publish() {
  if (configuration.metrics_publishing_type != NO_METRICS) {
    alfa_msg::msg::AlfaMetrics output_metrics;

    if (configuration.metrics_publishing_type & FULL_PROCESSING)
      output_metrics.metrics.push_back(full_processing_metric.message);

    if (configuration.metrics_publishing_type & HANDLER_PROCESSING) {
      output_metrics.metrics.push_back(handler_metric.message);
    }

    if (configuration.metrics_publishing_type & FRAMES_INFO) {
      output_metrics.metrics.push_back(number_of_processed_points.message);
    }

    if (configuration.metrics_publishing_type & DEBUG_POINTS) {
      for (unsigned int i = 0; i < configuration.number_of_debug_points; i++) {
        debug_points_message[i].metric = static_cast<float>(get_debug_point(i));

        output_metrics.metrics.push_back(debug_points_message[i]);
      }
    }
    publish_metrics(output_metrics);
  }
}

alfa_msg::msg::MetricMessage AlfaNode::get_metric_message(int metric) {
  switch (metric) {
    case HANDLER_TIME:
      return get_handler_time();
      break;

    case FULL_PROCESSING_TIME:
      return get_full_processing_time();
      break;

    default:
      return get_handler_time();
      break;
  }
  return get_handler_time();
}

alfa_msg::msg::MetricMessage AlfaNode::get_handler_time() {
#ifdef ALFA_VERBOSE
  verbose_info("get_handler_time", "getting handler time");
#endif
  return handler_metric.message;
}

alfa_msg::msg::MetricMessage AlfaNode::get_full_processing_time() {
#ifdef ALFA_VERBOSE
  verbose_info("get_full_processing_time", "getting full processing time");
#endif
  return full_processing_metric.message;
}