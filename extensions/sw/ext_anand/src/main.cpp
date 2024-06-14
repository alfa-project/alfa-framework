
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vector>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"
#include "alib_metrics.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_anand"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0
#define METRICS ALL_METRICS  // FULL_PROCESSING, HANDLER_PROCESSING

#define csize 0.5

/**
 * @brief Handler function that applies a distance filter to the received point
 * cloud.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN, node->get_input_pointcloud());
#else

  float zeta = node->get_extension_parameter("zeta");
  float epsilon = node->get_extension_parameter("epsilon");
  float delta = node->get_extension_parameter("delta");

  double x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;

  /* BUILD THE GRID */
  for (const auto &point : node->get_input_pointcloud()->points) {
    if (point.x < x_min) x_min = point.x;

    if (point.x > x_max) x_max = point.x;

    if (point.y < y_min) y_min = point.y;

    if (point.y > y_max) y_max = point.y;
  }

  int num_cells_x = std::ceil((x_max - x_min) / csize);
  int num_cells_y = std::ceil((y_max - y_min) / csize);

  std::vector<std::vector<std::vector<int> > > grid(
      num_cells_x, std::vector<std::vector<int> >(num_cells_y));

  int t = 0;

  for (const auto &point : node->get_input_pointcloud()->points) {
    int cell_x = std::floor((point.x - x_min) / csize);
    int cell_y = std::floor((point.y - y_min) / csize);

    if (cell_x == num_cells_x) cell_x -= 1;

    if (cell_y == num_cells_y) cell_y -= 1;

    grid[cell_x][cell_y].push_back(t++);

    // Setup the output point cloud
    node->push_point_output_pointcloud(point);
  }

  /* GRID ITERATION */
  for (int j = 0; j < num_cells_y; j++) {
    for (int i = 0; i < num_cells_x; i++) {
      if (!grid[i][j].empty()) {
        z_min = 10;
        z_max = 0;
        for (const auto index : grid[i][j])  // FIND CELL MIN AND MAX Z
        {
          AlfaPoint point = node->get_point_input_pointcloud(index);
          if (point.z < z_min) z_min = point.z;
          if (point.z > z_max) z_max = point.z;
        }

        /* CLASSIFICATION */

        double z_sub = z_max - z_min;
        double z_th = z_sub / 5;  // THRESHOLD

        if (z_min < zeta)  // Cell contains ground points
        {
          if (z_sub >= epsilon)  // Cell contains ground points, all of which
                                 // are below z_min + delta
          {
            for (const auto index : grid[i][j]) {
              node->set_custom_field_output_pointcloud(index,
                                                       ALFA_LABEL_NO_GROUND);
              AlfaPoint point = node->get_point_input_pointcloud(index);

              if (point.z < (z_min + delta))  // GROUND
              {
                node->set_custom_field_output_pointcloud(index,
                                                         ALFA_LABEL_GROUND);
              }
            }
          }

          else  // Cell contains a small object, with ground points being those
                // below z_min + z_th
          {
            for (const auto index : grid[i][j]) {
              node->set_custom_field_output_pointcloud(index, 0);
              AlfaPoint point = node->get_point_input_pointcloud(index);

              if (point.z < (z_min + z_th))  // GROUND
              {
                node->set_custom_field_output_pointcloud(index,
                                                         ALFA_LABEL_GROUND);
              }
            }
          }
        } else  // Cell does not contain ground points
        {
          for (const auto index : grid[i][j]) {
            node->set_custom_field_output_pointcloud(index,
                                                     ALFA_LABEL_NO_GROUND);
          }
        }
      }
    }
  }

#endif
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode *node) {
  // outside_points = 0;
#ifdef EXT_HARDWARE
  node->load_pointcloud(LOAD_STORE_CARTESIAN, node->get_output_pointcloud());
  ground_points = node->get_debug_point(0);
  outside_points = node->get_debug_point(1);
#endif

  AlfaGroundSegmentationMetrics metrics =
      evaluate_ground_segmentation_method(node, GROUND_LABELS);

  node->publish_pointcloud();
  node->set_debug_point(0, metrics.basic_metrics.number_input_points,
                        "Number of input points");
  node->set_debug_point(1, metrics.basic_metrics.number_output_points,
                        "Number of output points");
  node->set_debug_point(2, metrics.true_positive, "True positive");
  node->set_debug_point(3, metrics.false_positive, "False positive");
  node->set_debug_point(4, metrics.true_negative, "True negative");
  node->set_debug_point(5, metrics.false_negative, "False negative");
  node->set_debug_point(6, metrics.true_positive_rate, "True positive rate");
  node->set_debug_point(7, metrics.true_negative_rate, "True negative rate");
  node->set_debug_point(8, metrics.false_positive_rate, "False positive rate");
  node->set_debug_point(9, metrics.false_negative_rate, "False negative rate");
  node->set_debug_point(10, metrics.positive_predictive_value,
                        "Positive predictive value");
  node->set_debug_point(11, metrics.negative_predictive_value,
                        "Negative predictive value");
  node->set_debug_point(12, metrics.f1_score, "F1 score");
  node->set_debug_point(13, metrics.accuracy, "Accuracy");
  node->set_debug_point(14, metrics.IoUg, "IoUg");
  node->set_debug_point(15, metrics.number_input_ground_points,
                        "Number of ground points in the input point cloud");
  node->set_debug_point(16, metrics.number_output_ground_points,
                        "Number of ground points in the output point cloud");
}

/**
 * @brief Entry point of the program.
 *
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char **argv) {
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
  AlfaExtensionParameter parameters[30];

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 18;
  conf.metrics_publishing_type = METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_LABEL;

  parameters[0].parameter_name = "zeta";  // threshold
  parameters[0].parameter_value = -1.0;   //[m]

  parameters[1].parameter_name = "epsilon";  // threshold
  parameters[1].parameter_value = 0.5;       //[m] 0.5

  parameters[2].parameter_name = "delta";  // threshold
  parameters[2].parameter_value = 0.15;    //[m] 0.12

  // AnandMetrics.reset(
  //     new Metrics<PointType>(2, "Removed points", "Points outside of grid"));

  // rclcpp::on_shutdown(&callback_shutdown);

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(
      std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}