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

#include "alfa_node.hpp"


using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

// Global flag to signal threads to exit
std::atomic<bool> g_running(true);

// Signal handler for SIGINT
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << endl << "SIGINT received, exiting..." << std::endl;
    g_running = false;
  }
}

// Constructor
AlfaNode::AlfaNode(AlfaConfiguration conf, AlfaExtensionParameter parameters[10],
                   void (*handler_pointcloud)(AlfaNode *) = NULL,
                   void (*post_processing_pointcloud)(AlfaNode *) = NULL)
    : Node(conf.node_name),
      point_counter(0),
      handler_pointcloud(handler_pointcloud),
      post_processing_pointcloud(post_processing_pointcloud) {
  // Set the configuration and the signal handler
  configuration = conf;
  std::signal(SIGINT, signalHandler);

  // Print the characteristics of the node
  verbose_constr_chracteristics(conf.subscriber_topic, conf.node_name);

  // ROS2 QOS settings
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

#ifdef ALFA_VERBOSE
  verbose_begin("constructor");
#endif

  // If SIU is activated no subscriber is required to retrieve pointclouds
  if (!configuration.hardware_support.hardware_driver) {
    // Subscribe to the conf.subscriber_topic,
    pointcloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        conf.subscriber_topic, qos, std::bind(&AlfaNode::handler_callback, this, _1));
#ifdef ALFA_VERBOSE
    verbose_ok("constructor", "setup handler_callback");
#endif
  }

  // Create ALFA topics
  metrics_publisher = this->create_publisher<alfa_msg::msg::AlfaMetrics>(
      string(conf.node_name).append("_metrics"), qos);
  alive_publisher = this->create_publisher<alfa_msg::msg::AlfaAlivePing>(
      string(conf.node_name).append("_alive"), qos);
  pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      string(conf.node_name).append("_pointcloud"), qos);

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create ALFA topics");
#endif

  // Create the pointcloud deque
  ros_pointcloud.clear();

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "ros pointcloud");
#endif

  input_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);
  output_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create pcl pointclouds");
#endif

  metrics_setup();

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "set metrics");
#endif

  // Hardware setup
  if (configuration.hardware_support.hardware_extension ||
      configuration.hardware_support.hardware_driver)
    if (hardware_setup() == 1) {
      verbose_fail("constructor", "hardware_setup");
      abort();
      return;
    }

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "publisher");
#endif

  // Parameters config
  for (int i = 0; i < 10; i++) {
    if (parameters[i].parameter_name != "") {  // If the parameter is actually defined
      this->extension_parameters.push_back(parameters[i]);
      this->declare_parameter(parameters[i].parameter_name, (double)parameters[i].parameter_value);
      if (configuration.hardware_support.hardware_extension ||
          configuration.hardware_support.hardware_driver) {
        unit_write_register(UNIT_USER_DEFINE_0 + i * 4,
                            parameters[i].parameter_value * FIXED_POINT_MULTIPLIER);
      }
    }
  }
  this->timeout_counter = 0;

  // Threads setup
  pointcloud_publisher_thread =
      new std::thread(&AlfaNode::pointcloud_publisher_thread_handler, this);
  ticker_thread = new std::thread(&AlfaNode::ticker_alive, this);
  alfa_main_thread = new std::thread(&AlfaNode::alfa_main_thread_handler, this);

  if (setup_thread(alfa_main_thread, 1) != 0) return;
  if (setup_thread(pointcloud_publisher_thread, 2) != 0) return;
  if (setup_thread(ticker_thread, 3) != 0) return;

  // Threads are detached
  pointcloud_publisher_thread->detach();
  ticker_thread->detach();
  alfa_main_thread->detach();
#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create threads");
  verbose_end("constructor");
#endif
}

AlfaNode::~AlfaNode() {
  delete this->ticker_thread;
  delete this->alfa_main_thread;
  delete this->pointcloud_publisher_thread;

  if (configuration.hardware_support.hardware_extension ||
      configuration.hardware_support.hardware_driver) {
    close(ext_fd);
    close(mem_fd);
    close(ext_mem_fd);
  }
}

int AlfaNode::setup_thread(std::thread *thread, size_t CPU) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(CPU, &cpuset);

  int result = pthread_setaffinity_np(thread->native_handle(), sizeof(cpu_set_t), &cpuset);
  if (result != 0) {
    std::cerr << "Error setting thread affinity: " << strerror(result) << std::endl;
    return 1;
  }

  sched_param sch;
  int policy;
  pthread_getschedparam(thread->native_handle(), &policy, &sch);
  sch.sched_priority = 90;  // Set the priority, 99 is the maximum for SCHED_FIFO
  result = pthread_setschedparam(thread->native_handle(), SCHED_FIFO, &sch);
  if (result != 0) {
    std::cerr << "Failed to set thread scheduling: " << strerror(result) << std::endl;
    return 1;
  }

  return 0;
}

void AlfaNode::handler_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  if ((cloud->width * cloud->height) == 0) {
    return;
  } else {
    if (ros_pointcloud.size() > 3) {
      ros_pointcloud_mutex.lock();
      ros_pointcloud.pop_front();
      ros_pointcloud_mutex.unlock();
    } else {
      std::lock_guard<std::mutex> lk(ros_pointcloud_condition_mutex);
      ros_pointcloud_mutex.lock();
      ros_pointcloud.push_back(*cloud);
      ros_pointcloud_mutex.unlock();
      ros_pointcloud_condition.notify_one();
    }

#ifdef ALFA_VERBOSE
    verbose_info("handler_callback", "Received a point cloud with: " +
                                         std::to_string(cloud->width * cloud->height) + " size");
#endif
  }
}

void AlfaNode::alfa_main_thread_handler() {
  while (rclcpp::ok() && g_running) {
    std::unique_lock<std::mutex> lk(ros_pointcloud_condition_mutex);
    ros_pointcloud_condition.wait(lk, [this] { return !ros_pointcloud.empty() || !g_running; });
    if (!g_running) {
      return;
    }

#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "New pointcloud received");
#endif
    full_processing_metric.start = high_resolution_clock::now();
#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "Converting ros2 msg to pcl");
#endif
    convert_msg_to_pointcloud();
    this->point_counter = 0;
    handler_metric.start = high_resolution_clock::now();
    output_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);
#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "Calling user-defined handler");
#endif
    (*handler_pointcloud)(this);

    if (!configuration.hardware_support.hardware_extension) {
      handler_metric.stop = high_resolution_clock::now();

#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "Calling user-defined post processing");
#endif
      (*post_processing_pointcloud)(this);
#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "Return from user-defined post processing");
#endif
      full_processing_metric.stop = high_resolution_clock::now();
    } else {
      __sync_synchronize();
      unit_write_register(UNIT_SIGNALS_SOFTWARE_FRAME_DONE, 0);
      unit_write_register(UNIT_SIGNALS_SOFTWARE_POINTCLOUD_READY, 1);
#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "calling hardware processing");
#endif
      unit_wait_for_value(UNIT_SIGNALS_HARDWARE_PROCESSING_DONE, 1);
      unit_write_register(UNIT_SIGNALS_SOFTWARE_POINTCLOUD_READY, 0);
      handler_metric.stop = high_resolution_clock::now();

#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "done");
      verbose_info("main_thread", "calling post processing");
#endif
      (*post_processing_pointcloud)(this);
      unit_write_register(UNIT_SIGNALS_SOFTWARE_FRAME_DONE, 1);
      full_processing_metric.stop = high_resolution_clock::now();
    }
    metrics_update();
    metrics_publish();
  }
}

void AlfaNode::ticker_alive() {
  while (rclcpp::ok() && g_running) {
    alfa_msg::msg::AlfaAlivePing new_ping;
    new_ping.node_name = (string)this->get_name();
    new_ping.node_type = "extension";
    new_ping.config_service_name = (string)this->get_name() + "_settings";
    new_ping.config_tag = "Configuration";

    new_ping.default_configurations.clear();
    for (AlfaExtensionParameter parameter : this->extension_parameters) {
      alfa_msg::msg::ConfigMessage msg_parameter;

      msg_parameter.config_name = parameter.parameter_name;
      msg_parameter.config = parameter.parameter_value;

      new_ping.default_configurations.push_back(msg_parameter);
    }

    new_ping.current_status = (int)(this->configuration.hardware_support.hardware_driver +
                                    this->configuration.hardware_support.hardware_extension);
    alive_publisher->publish(new_ping);
    std::this_thread::sleep_for(std::chrono::milliseconds(ALIVE_TIMER_SLEEP));
  }
  ros_pointcloud_condition.notify_one();
  pcl2_frame_condition.notify_one();
  cout << endl << "ALFA Extension Shutdown..." << endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
}

void AlfaNode::pointcloud_publisher_thread_handler() {
  while (rclcpp::ok() && g_running) {
    std::unique_lock<std::mutex> lk(pcl2_frame_condition_mutex);
    pcl2_frame_condition.wait(lk, [this] { return !pcl2_frame.empty() || !g_running; });

    if (!g_running) {
      return;
    }

    auto temp = high_resolution_clock::now();

    auto latency = std::chrono::milliseconds(configuration.latency);
    pcl2_frame_mutex.lock();
    auto pcl2_frame_temp = pcl2_frame.front();
    pcl2_frame.pop_front();
    pcl2_frame_mutex.unlock();

    /*while (start_temp + latency > high_resolution_clock::now())
    {
            std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }*/
    publish_pointcloud(pcl2_frame_temp);
    publishing_metric.start = temp;
    publishing_metric.stop = high_resolution_clock::now();
  }
}

void AlfaNode::set_multi_thread(int n_threads, void (*func)(AlfaNode *), AlfaNode *ptr) {
  vector<std::thread *> thread_list;
  thread_list.clear();

  for (int i = 0; i < n_threads; i++) thread_list.push_back(new std::thread(func, ptr));

  for (int i = 0; i < n_threads; i++) thread_list[i]->join();
}
