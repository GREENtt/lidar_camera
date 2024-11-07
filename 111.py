import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
 
class LidarCameraFuser(Node):
    def __init__(self):
        super().__init__('lidar_camera_fuser')
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.last_lidar_time = None
        self.last_camera_time = None
 
    def lidar_callback(self, msg):
        self.last_lidar_time = msg.header.stamp
        self.synchronize()
 
    def camera_callback(self, msg):
        self.last_camera_time = msg.header.stamp
        self.synchronize()
 
    def synchronize(self):
        if self.last_lidar_time is None or self.last_camera_time is None:
            return
        
        if self.last_lidar_time > self.last_camera_time:
            # Lidar is newer, wait for Camera to catch up
            return
        elif self.last_camera_time > self.last_lidar_time:
            # Camera is newer, wait for Lidar to catch up
            return
        
        # Both messages are synchronized, process them
        self.fuse_data()
 
    def fuse_data(self):
        # Your data fusing logic here
        self.get_logger().info('Data fused successfully')
 
def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 

  

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
 
class TimedSynchronizer(Node):
    def __init__(self):
        super().__init__('timed_synchronizer')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
 
    def pointcloud_callback(self, msg):
        self.last_pointcloud_header = msg.header
        self.process_data()
 
    def image_callback(self, msg):
        self.last_image_header = msg.header
        self.process_data()
 
    def process_data(self):
        if hasattr(self, 'last_pointcloud_header') and hasattr(self, 'last_image_header'):
            if self.last_pointcloud_header.stamp == self.last_image_header.stamp:
                self.get_logger().info('Processing data with timestamp: {0}'.format(self.last_pointcloud_header.stamp.sec))
                # Do your bingchuli here
                # ...
                # Remove the headers to avoid processing again
                delattr(self, 'last_pointcloud_header')
                delattr(self, 'last_image_header')
 
def main(args=None):
    rclpy.init(args=args)
    synchronizer = TimedSynchronizer()
    rclpy.spin(synchronizer)
    rclpy.shutdown()
 
 
 
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
 
class TimestampSync(Node):
    def __init__(self):
        super().__init__('timestamp_sync')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10)
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
 
    def pointcloud_callback(self, msg):
        self.last_pointcloud_header = msg.header
        self.align_timestamps(msg)
 
    def image_callback(self, msg):
        self.last_image_header = msg.header
        self.align_timestamps(msg)
 
    def align_timestamps(self, msg):
        if self.last_pointcloud_header is not None and self.last_image_header is not None:
            pointcloud_time = Time(self.last_pointcloud_header.stamp)
            image_time = Time(self.last_image_header.stamp)
            time_diff = pointcloud_time - image_time
 
            if time_diff.nanoseconds > 0:
                self.get_logger().info('PointCloud is ahead of Image by %d ns' % time_diff.nanoseconds)
                # Synchronize the timestamps
                msg.header.stamp = self.last_image_header.stamp
                self.publish(msg)
            elif time_diff.nanoseconds < 0:
                self.get_logger().info('PointCloud is behind Image by %d ns' % -time_diff.nanoseconds)
                # Do not publish if PointCloud is behind Image
                pass
            else:
                self.get_logger().info('PointCloud and Image are synchronized')
                self.publish(msg)
 
    def publish(self, msg):
        # Replace with your publisher
        pass
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TimestampSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
