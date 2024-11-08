# ALFA-Node

<p align="justify"> <b> ALFA Node</b>  is a ROS-based node that supports the ALFA Extensions to facilitate the development of point cloud processing algorithms. The ALFA Node receives configurations through the ROS2 set parameters command (<b>ros2 param set &ltnode_name&gt &ltparameter_name&gt &ltvalue&gt</b>), outputs real-time metrics (<b>/NODE_NAME_metrics</b>) and publishes the processed point cloud data into a new <b>PointCloud2</b> ROS2 topic (<b>NODE_NAME_pointcloud</b>). The point cloud data can be retrieved from a real LiDAR sensor connected to the platform or from a public dataset thtough a ROS2 Bag (rosbag tool). Additionally, the Desktop version of the framework supports the <b>ALFA Monitor</b>, a GUI tool specially designed to configure the ALFA Extension, visualize point cloud data, and read real-time metrics from every ALFA Node running both in the Desktop and the Embedded System.</p></a>

**Note:** Changing this package must be done carefully, as it may affect the ALFA Monitor and other ALFA Extensions.

## File Structure
- **include/alfa_node.hpp:** Contains the ALFA Node class definition.
- **include/alfa_defines.hpp:** Contains the ALFA Node defines.
- **include/alfa_structs.hpp:** Contains the ALFA Node supporting structures.
- **src/alfa_node.cpp:** Contains the source file of the ALFA Node class.

## Class: AlfaNode

The AlfaNode class inherits from rclcpp::Node and provides several methods and members to manage point cloud data, data processing, and metrics.

### Public Members and Methods

- **Constructor and Destructor:**
  - AlfaNode(AlfaConfiguration conf, AlfaExtensionParameter *parameters, void (*handler_pointcloud)(AlfaNode *), void (*post_processing_pointcloud)(AlfaNode *))
  - ~AlfaNode()

- **Parameter Handling:**
  - float get_extension_parameter(string parameter_name)

- **Metric Functions:**
  - alfa_msg::msg::MetricMessage get_metric_message(int metric)

- **Publishing Methods:**
  - void publish_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr, std::uint32_t latency = 0)
  - void publish_metrics(alfa_msg::msg::AlfaMetrics &metrics)

- **Pointcloud Manipulation:**
  - pcl::PointCloud<AlfaPoint>::Ptr get_input_pointcloud()
  - pcl::PointCloud<AlfaPoint>::Ptr get_output_pointcloud()
  - vector<AlfaPoint> get_input_pointcloud_as_vector()
  - vector<AlfaPoint> get_output_pointcloud_as_vector()
  - std::uint32_t get_input_pointcloud_size()
  - std::uint32_t get_output_pointcloud_size()
  - bool is_input_pointcloud_empty()
  - bool is_output_pointcloud_empty()
  - bool is_last_input_pointcloud_point()
  - void push_point_output_pointcloud(AlfaPoint point)
  - bool get_point_input_pointcloud(std::uint32_t position, AlfaPoint &point)
  - AlfaPoint get_point_input_pointcloud(std::uint32_t position)
  - AlfaPoint get_point_output_pointcloud(std::uint32_t position)
  - bool get_point_input_pointcloud(AlfaPoint &point)
  - bool reset_input_pointcloud_counter()
  - bool set_custom_field_output_pointcloud(std::uint32_t position, std::uint32_t custom_field)

- **Hardware Store and Load:**
  - void store_pointcloud(int type, pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr)
  - void load_pointcloud(int type, pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr)

- **Extension Memory:**
  - void read_ext_memory(uint32_t offset, size_t size, void *buffer)

- **Multithreading:**
  - void set_multi_thread(int n_threads, void (*func)(AlfaNode *), AlfaNode *)

- **Debugging:**
  - float get_debug_point(std::uint16_t)
  - void set_debug_point(std::uint16_t, float, string)

### Private Members

- **Point Clouds:**
  - pcl::PointCloud<AlfaPoint>::Ptr input_pointcloud, output_pointcloud
  - AlfaConfiguration configuration
  - vector<AlfaExtensionParameter> extension_parameters
  - std::uint32_t point_counter
  - std::uint32_t timeout_counter

- **ROS Subscribers and Publishers:**
  - rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber
  - rclcpp::Publisher<alfa_msg::msg::AlfaMetrics>::SharedPtr metrics_publisher
  - rclcpp::Publisher<alfa_msg::msg::AlfaAlivePing>::SharedPtr alive_publisher
  - rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher

- **Files and Memory Mapping:**
  - int ext_fd, mem_fd, ext_mem_fd
  - off_t pointcloud_ptr_address
  - AlfaPointcloud pointcloud
  - std::uint64_t *ext_mem

- **Threads:**
  - std::thread *ticker_thread, *pointcloud_publisher_thread, *alfa_main_thread

- **Mutexes and Condition Variables:**
  - std::mutex input_mutex, output_mutex, output_counter_mutex, ros_pointcloud_mutex, pcl2_frame_mutex, ros_pointcloud_condition_mutex, pcl2_frame_condition_mutex
  - std::condition_variable ros_pointcloud_condition, pcl2_frame_condition

- **Metrics:**
  - AlfaMetric handler_metric, full_processing_metric, publishing_metric, number_of_processed_points
  - alfa_msg::msg::MetricMessage debug_points_message[20]
  - std::deque<sensor_msgs::msg::PointCloud2> pcl2_frame
  - std::deque<sensor_msgs::msg::PointCloud2> ros_pointcloud

- **Parameters Callback:**
  - OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle

### Private Methods

- **Unit Register Functions:**
  - void unit_write_register(unsigned int offset, unsigned int value)
  - unsigned int unit_read_register(unsigned int offset)
  - void unit_wait_for_value(unsigned int offset, unsigned int value)

- **Thread Handlers:**
  - int setup_thread(std::thread *thread, size_t CPU)
  - void pointcloud_publisher_thread_handler()
  - void alfa_main_thread_handler()
  - void ticker_alive()

- **Conversion Functions:**
  - void convert_msg_to_pointcloud()

- **Callbacks:**
  - void handler_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)

- **User-defined Functions:**
  - void (*handler_pointcloud)(AlfaNode *)
  - void (*post_processing_pointcloud)(AlfaNode *)

- **Hardware Setup:**
  - bool hardware_setup()

- **Metrics:**
  - void metrics_setup()
  - void metrics_update()
  - void metrics_publish()
  - alfa_msg::msg::MetricMessage get_handler_time()
  - alfa_msg::msg::MetricMessage get_full_processing_time()

- **ROS Publishing Helper:**
  - void publish_pointcloud(sensor_msgs::msg::PointCloud2 &pointcloud)

- **Hardware Storage and Loading:**
  - void store_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud)
  - void load_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud)

- **Verbose Functions:**
  - void verbose_constr_chracteristics(string, string)
  - void verbose_begin(string)
  - void verbose_end(string)
  - void verbose_ok(string, string)
  - void verbose_fail(string, string)
  - void verbose_info(string, string)
  - void verbose_not_defined(string)

## Usage

To use the AlfaNode class, instantiate it with the required configuration and handlers. The node will automatically handle the subscription to point cloud messages, process them, and publish the results along with metrics.