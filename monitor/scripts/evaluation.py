import os
import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from alfa_msg.msg import AlfaMetrics, MetricMessage
from scipy.spatial import KDTree
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
import time


class PointCloudEvaluator(Node):
    def __init__(self, pointcloud_topic, additional_topic, frame_count, bag_file_path, evaluation_type, bounding_box=None):
        super().__init__('pointcloud_evaluator')

        # QoS profile setup
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.publisher = self.create_publisher(PointCloud2, pointcloud_topic, qos_profile)
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            qos_profile)
        self.additional_subscriber = self.create_subscription(
            PointCloud2,
            additional_topic,
            self.additional_callback,
            qos_profile)

        metrics_topic = additional_topic.replace("_pointcloud", "_metrics")

        self.metrics_subscriber = self.create_subscription(
            AlfaMetrics,
            metrics_topic,
            self.metrics_callback,
            qos_profile)

        self.user_defined_frame_count = frame_count
        self.evaluation_type = evaluation_type
        self.bounding_box = bounding_box  # Store bounding box for evaluation
        self.current_frame_count = 0
        self.processed_frames = 0  # Track frames processed for progress
        self.widths = []
        self.widths_output = []
        self.handler_times = []
        self.full_times = []
        self.errors = []
        self.original_pointcloud = []  # FIFO buffer for original pointclouds
        self.additional_pointcloud = []  # FIFO buffer for additional pointclouds
        self.bag_file_path = bag_file_path
        self.bag_reader = None
        self.evaluation_started = False  # Flag to track when evaluation starts
        self.ready_to_process = False  # Flag to start processing after flushing old messages
        self.flushing_old_messages = True  # Flag to discard old messages
        self.open_bag()

        # Flush old messages before publishing the first frame
        self.flush_old_messages()

    def open_bag(self):
        storage_options = StorageOptions(uri=self.bag_file_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        # Suppress console output
        original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

        try:
            self.bag_reader = SequentialReader()
            self.bag_reader.open(storage_options, converter_options)
        finally:
            sys.stdout.close()
            sys.stdout = original_stdout

    def flush_old_messages(self):
        """Flushes old messages by discarding all received messages until ready to process."""
        self.create_timer(2.0, self.start_publishing_first_frame)

    def start_publishing_first_frame(self):
        """Publishes the first frame after flushing old messages."""
        self.flushing_old_messages = False  # Stop ignoring messages
        self.ready_to_process = True  # Start processing after this point
        self.publish_frame()

    def publish_frame(self):
        PointCloud2 = get_message('sensor_msgs/msg/PointCloud2')

        while self.bag_reader.has_next():
            topic, data, t = self.bag_reader.read_next()

            # Only process /velodyne_points topic
            if topic == '/velodyne_points':
                msg = deserialize_message(data, PointCloud2)
                self.publisher.publish(msg)
                self.evaluation_started = True  # Mark evaluation as started after first frame is published
                return  # Exit the function after publishing the frame

        self.open_bag()
        self.publish_frame()

    def pointcloud_callback(self, msg):
        if self.flushing_old_messages:
            return

        if msg.width > 0:
            self.original_pointcloud.append(msg)  # FIFO buffer for original pointclouds
            self.widths.append(msg.width)  # Add number of points to widths list
            self.match_pointclouds()

    def additional_callback(self, msg):
        if self.flushing_old_messages:
            return

        if msg.width > 0 and self.ready_to_process:  # Only consider output frames after system is ready to process
            self.additional_pointcloud.append(msg)  # FIFO buffer for additional pointclouds
            self.widths_output.append(msg.width)  # Add number of output points to widths_output list
            self.match_pointclouds()

    def match_pointclouds(self):
        if len(self.original_pointcloud) > 0 and len(self.additional_pointcloud) > 0:
            original_msg = self.original_pointcloud.pop(0)
            additional_msg = self.additional_pointcloud.pop(0)

            if self.evaluation_type == 2 or self.evaluation_type == 3:
                self.analyze_pointcloud_difference(original_msg, additional_msg)

            self.processed_frames += 1
            self.check_frame_count()

    def analyze_pointcloud_difference(self, pointcloud1_msg, pointcloud2_msg):
        points1 = np.array(list(read_points(pointcloud1_msg, field_names=("x", "y", "z"), skip_nans=True)))
        points2 = np.array(list(read_points(pointcloud2_msg, field_names=("x", "y", "z"), skip_nans=True)))

        if len(points1) == 0 or len(points2) == 0:
            raise ValueError("Point clouds are empty")

        points1_unstructured = structured_to_unstructured(points1).astype(np.float64)
        points2_unstructured = structured_to_unstructured(points2).astype(np.float64)

        # Apply bounding box if specified
        if self.bounding_box:
            points1_unstructured = self.filter_by_bounding_box(points1_unstructured)
            points2_unstructured = self.filter_by_bounding_box(points2_unstructured)

        tree = KDTree(points2_unstructured)
        squared_errors = [tree.query(point)[0] ** 2 for point in points1_unstructured]
        mse = np.mean(squared_errors)

        max_value = np.max(np.linalg.norm(points1_unstructured, axis=1))
        psnr = self.calculate_psnr(mse, max_value)
        self.errors.append(psnr)

    def filter_by_bounding_box(self, points):
        """Filters points to only include those inside the bounding box."""
        min_x, max_x, min_y, max_y, min_z, max_z = self.bounding_box
        return points[(points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                      (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &
                      (points[:, 2] >= min_z) & (points[:, 2] <= max_z)]

    def calculate_psnr(self, mse, max_value):
        if mse == 0:
            return float('inf')  # Keep PSNR as infinity for identical point clouds
        return 20 * np.log10(max_value / np.sqrt(mse))

    def metrics_callback(self, msg):
        """Callback to handle incoming metrics messages."""
        for metric_msg in msg.metrics:
            if metric_msg.metric > 0:  # Ignore negative values and zeros
                if metric_msg.metric_name == 'Handler processing time':
                    self.handler_times.append(metric_msg.metric)
                elif metric_msg.metric_name == 'Full processing time':
                    self.full_times.append(metric_msg.metric)

    def check_frame_count(self):
        self.print_progress_bar(self.processed_frames, self.user_defined_frame_count)
        if self.processed_frames >= self.user_defined_frame_count:
            self.write_evaluation()
            self.destroy_node()

    def print_progress_bar(self, iteration, total, length=50):
        if iteration == 1:
            self.start_time = time.time()
        percent = ("{0:.1f}").format(100 * (iteration / float(total)))
        filled_length = int(length * iteration // total)
        bar = '█' * filled_length + '-' * (length - filled_length)
        print(f'\rProgress: |{bar}| {percent}% Complete', end='\r')
        if iteration == total:
            print()

    def write_evaluation(self):
        summary_str = self.calculate_summary()
        print(summary_str)
        print("All frames processed.")

    def calculate_stats(self, data):
        filtered_data = [value for value in data if value > 0]
        if not filtered_data:
            return {'Mean': None, 'Standard deviation': None, 'Max': None, 'Min': None}
        return {
            'Mean': np.mean(filtered_data),
            'Standard deviation': np.std(filtered_data),
            'Max': np.max(filtered_data),
            'Min': np.min(filtered_data)
        }

    def calculate_summary(self):
        summary_str = "α Evaluation Summary:\n"
        fields = {}
        match self.evaluation_type:
            case 1:
                fields = {
                    'Number of points': self.widths,
                    'Number of output points': self.widths_output,
                    'Handler Processing Time': self.handler_times,
                    'Full Processing Time': self.full_times
                }
            case 2:
                fields = {
                    'PSNR': self.errors
                }
            case 3:
                fields = {
                    'Number of points': self.widths,
                    'Number of output points': self.widths_output,
                    'Handler Processing Time': self.handler_times,
                    'Full Processing Time': self.full_times,
                    'PSNR': self.errors
                }
            case _:
                return "Invalid evaluation type"

        for key, data in fields.items():
            stats = self.calculate_stats(data)
            summary_str += f"  - {key}:\n"
            for stat_name, value in stats.items():
                value_str = "Data not available" if value is None else f"{value:.3f}" if isinstance(value, float) else str(value)
                summary_str += f"         - {stat_name}: {value_str}\n"

        return summary_str


def get_ros2_topics(node: Node):
    topic_tuples = node.get_topic_names_and_types()
    topics = [topic[0] for topic in topic_tuples]
    return topics


def main(args=None):
    rclpy.init(args=args)

    print("Welcome to the α evaluation script.")
    print("Select the type of evaluation:")
    print("1. Timing Evaluation")
    print("2. Distortion Evaluation")
    print("3. Full Evaluation")
    evaluation_type = int(input("Enter the evaluation type by number: "))

    bounding_box = None

    # Ask for bounding box if evaluation type is 2 or 3
    if evaluation_type == 2 or evaluation_type == 3:
        print("Would you like to apply a Bounding Box (BB)?")
        print("0: Use original point cloud (no bounding box)")
        print("1: Apply a default 100x100x100 bounding box")
        print("2: Set a custom bounding box")
        bb_choice = int(input("Enter your choice by number: "))

        if bb_choice == 1:
            bounding_box = (-50, 50, -50, 50, -50, 50)  # Default 100x100x100 BB centered at (0, 0, 0)
        elif bb_choice == 2:
            print("Define the Bounding Box in the format: min_x max_x min_y max_y min_z max_z")
            bounding_box = tuple(map(float, input("Enter bounding box (space-separated values): ").split()))

    print("\nSelect the extension topic to evaluate.")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    all_topics = get_ros2_topics(rclpy.create_node('topic_list_node'))

    extention_topics = [topic for topic in all_topics if topic.startswith('/ext_') and topic.endswith('_pointcloud')]
    if not extention_topics:
        extention_topic = '/' + input("Enter the extension name: ") + '_pointcloud'
    else:
        print("Available extension topics:")
        for i, topic in enumerate(extention_topics):
            print(f"{i+1}: {topic}")

        extension_topic_index = int(input("Select the extension topic by number: ")) - 1
        if extension_topic_index <= -1:
            return
        extention_topic = extention_topics[extension_topic_index]

    print("Select the bag file(s) used in the evaluation. Separate indices with commas (e.g., 1,2,3).")
    bag_file_path = os.path.join(script_dir, '../bags')
    list_of_files = os.listdir(bag_file_path)
    for i, file in enumerate(list_of_files):
        print(f"{i+1}: {file}")  # Bag index starts at 1

    selected_bags = input("Select the bag file(s) by number (comma-separated): ")
    selected_bag_indices = [int(i) - 1 for i in selected_bags.split(',') if i.isdigit()]  # Adjust for 1-based indexing

    frame_count = int(input("Enter the number of frames to evaluate (100-1000): "))
    frame_count = max(100, min(frame_count, 1000))

    # Create the executor once, reuse it
    executor = rclpy.executors.SingleThreadedExecutor()

    for bag_index in selected_bag_indices:
        if bag_index < 0 or bag_index >= len(list_of_files):
            print(f"Invalid bag index: {bag_index + 1}")
            continue

        # Construct the path to the subdirectory and the corresponding .db3 file
        bag_directory = os.path.join(bag_file_path, list_of_files[bag_index])
        bag_file = os.path.join(bag_directory, list_of_files[bag_index] + '.db3')

        print(f"Running evaluation on bag {list_of_files[bag_index]}")

        # Reinitialize the evaluator node for each bag
        evaluator = PointCloudEvaluator('/velodyne_points', extention_topic, frame_count, bag_file, evaluation_type, bounding_box)
        executor.add_node(evaluator)

        # Spin the executor until the current bag's evaluation is finished
        while evaluator.processed_frames < frame_count:
            executor.spin_once(timeout_sec=0.1)

        # Remove node after processing the bag
        executor.remove_node(evaluator)

    # Shutdown the executor after all bags have been processed
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
