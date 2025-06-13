import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from alfa_msg.msg import AlfaMetrics, MetricMessage  # Importing custom message types
from scipy.spatial import KDTree
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import time

class PointCloudEvaluator(Node):
    def __init__(self, executor, pointcloud_topic, additional_topic, frame_count, bag_file_path):
        super().__init__('pointcloud_evaluator')
        self.executor = executor
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10)
        self.metrics_subscriber = self.create_subscription(
            AlfaMetrics,
            metrics_topic,
            self.metrics_callback,
            10)
        
        # Determine the name of the additional topic
        additional_topic = metrics_topic.replace("_metrics", "_pointcloud")

        self.additional_subscriber = self.create_subscription(
            PointCloud2,
            additional_topic,
            self.additional_callback,
            10)
        self.user_defined_frame_count = frame_count
        self.current_frame_count = 0
        self.widths = []
        self.widths_output = []
        self.handler_times = []
        self.full_times = []
        self.delays = []
        self.pointcloud_times = []
        self.ext_pointcloud_times = []

    def pointcloud_callback(self, msg):
        if msg.width > 0:  # Ignore negative values and zeros
            self.widths.append(msg.width)
        self.check_frame_count()
        self.pointcloud_times.append(time.time_ns())

    def additional_callback(self, msg):
        if msg.width > 0:  # Ignore negative values and zeros
            self.widths_output.append(msg.width)
        self.ext_pointcloud_times.append(time.time_ns())

    def metrics_callback(self, msg):
        for metric_msg in msg.metrics:
            if metric_msg.metric > 0:  # Ignore negative values and zeros
                if metric_msg.metric_name == 'Handler processing time':
                    self.handler_times.append(metric_msg.metric)
                elif metric_msg.metric_name == 'Full processing time':
                    self.full_times.append(metric_msg.metric)

    def calculate_delay(self):
        # Calculate the delay if both timestamps are available
        while len(self.pointcloud_times) > 0 and len(self.ext_pointcloud_times) > 0:
            # Get the oldest timestamps from each list
            pointcloud_time = self.pointcloud_times.pop(0)
            ext_pointcloud_time = self.ext_pointcloud_times.pop(0)

            # Check for potential overflow and correct if necessary
            if ext_pointcloud_time < pointcloud_time:
                # Assuming the overflow happens at 2^64 for nanoseconds
                # Add the overflow offset to the ext_pointcloud_time
                ext_pointcloud_time += 2**64

            # Calculate delay in nanoseconds
            delay_ns = ext_pointcloud_time - pointcloud_time
            # Convert nanoseconds to microseconds
            delay_us = delay_ns / 1000
            self.delays.append(delay_us)

    def check_frame_count(self):
        self.current_frame_count += 1
        self.print_progress_bar(self.current_frame_count, self.user_defined_frame_count)
        if self.current_frame_count >= self.user_defined_frame_count:
            self.write_evaluation()
            self.destroy_node()

    def print_progress_bar(self, iteration, total, length=50):
        percent = ("{0:.1f}").format(100 * (iteration / float(total)))
        filled_length = int(length * iteration // total)
        bar = '█' * filled_length + '-' * (length - filled_length)
        print(f'\rProgress: |{bar}| {percent}% Complete', end='\r')
        # Print New Line on Complete
        if iteration == total: 
            print()

    def write_evaluation(self):
        summary_str = self.calculate_summary()
        print(summary_str)
        print("All frames processed. Exiting the script.")
        # Destroy the node explicitly
        self.executor.shutdown()
        # Destroy the node explicitly
        self.destroy_node()
        # Shutdown ROS to clean up any remaining resources.
        rclpy.shutdown()
        # Use exit(0) to exit the script explicitly
        exit(0)

    def calculate_summary(self):
        # Function to calculate summary statistics
        def calculate_stats(data):
            filtered_data = [value for value in data if value > 0]
            if not filtered_data:
                return {'Mean': None, 'Standard deviation': None, 'Max': None, 'Min': None}
            else:
                return {
                    'Mean': np.mean(filtered_data),
                    'Standard deviation': np.std(filtered_data),
                    'Max': np.max(filtered_data),
                    'Min': np.min(filtered_data)
                }

        # Calculate summary for each field
        widths_stats = calculate_stats(self.widths)
        widths_output_stats = calculate_stats(self.widths_output)
        handler_stats = calculate_stats(self.handler_times)
        full_stats = calculate_stats(self.full_times)

        # Create a formatted string for the summary
        summary_str = "α Evaluation Summary:\n"
        fields = {
            'Number of points': widths_stats,
            'Number of output points': widths_output_stats,
            'Handler Processing Time': handler_stats,
            'Full Processing Time': full_stats,
            #'Time between topic outputs': delay_stats
        }
        for key, stats in fields.items():
            summary_str += f"  - {key}:\n"
            for stat_name, value in stats.items():
                if value is not None:
                    summary_str += f"         - {stat_name}: {value:.1f}\n"
                else:
                    summary_str += f"         - {stat_name}: Data not available\n"
        return summary_str

def get_ros2_topics(node: Node):
    # Get the list of topics from the ROS2 graph
    topic_tuples = node.get_topic_names_and_types()
    # Extract topic names from the tuples
    topics = [topic[0] for topic in topic_tuples]
    return topics

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = rclpy.create_node('topic_list_node')
    
    # Get the list of all topics
    all_topics = get_ros2_topics(node)
    
    # Filter out topics that start with '/ext_', '/rosout', and '/parameter_events'
    filtered_topics = [topic for topic in all_topics if not topic.startswith('/ext_') and topic not in ('/rosout', '/parameter_events')]
    
    # Display the filtered list of topics to the user
    print("Available topics:")
    for i, topic in enumerate(filtered_topics):
        print(f"{i+1}: {topic}")
    
    # User selects pointcloud topic
    pointcloud_topic_index = int(input("Select the pointcloud topic by number (0 to exit): ")) - 1
    if pointcloud_topic_index == -1:
        print("Exiting the script.")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    pointcloud_topic = filtered_topics[pointcloud_topic_index]
    
    # Filter and list metrics topics
    metrics_topics = [topic for topic in all_topics if topic.startswith('/ext_') and topic.endswith('_metrics')]
    print("Available metrics topics:")
    for i, topic in enumerate(metrics_topics):
        print(f"{i+1}: {topic}")
    
    # User selects metrics topic
    metrics_topic_index = int(input("Select the metrics topic by number: ")) - 1
    if metrics_topic_index <= -1:
        print("Exiting the script.")
        node.destroy_node()
        rclpy.shutdown()
        return
    metrics_topic = metrics_topics[metrics_topic_index]
    
    # User defines number of frames to evaluate
    frame_count = int(input("Enter the number of frames to evaluate (100-100000): "))
    frame_count = max(100, min(frame_count, 100000))
    
    # Clean up the topic list node
    node.destroy_node()
    
     # Create and run the evaluator node with the executor
    evaluator = PointCloudEvaluator(executor, pointcloud_topic, metrics_topic, frame_count)
    executor.add_node(evaluator)
    executor.spin()

if __name__ == '__main__':
    main()
