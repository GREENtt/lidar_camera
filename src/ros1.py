import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as point_cloud2
import threading
 
# 定义一个ROS 2的节点类，用于订阅点云数据并处理
class PointCloudSubscriber(Node):
    def __init__(self, topic):
        # 调用Node的构造函数
        super().__init__('point_cloud_subscriber')
        
        # 创建订阅者，订阅特定的点云话题
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.point_cloud_callback, 10)
        
        # 防止未使用的变量警告
        self.subscription
        
        # 日志信息：节点启动并订阅指定话题
        self.get_logger().info(f'点云订阅者的主题已经开始: {topic}')
        
        # 标记点云是否已经保存
        self.saved = False
        
         # 创建一个事件对象，用于线程间的同步
        self.save_event = threading.Event()
        self.data = []
       
 
    def point_cloud_callback(self, msg):
        # 当收到点云消息时，这个回调函数会被调用
        try:
            # 将ROS点云消息转换为Numpy数组，包括强度字段
            #point_list = []
            #for data in point_cloud2.read_points(msg, field_names=['x', 'y', 'z', "intensity"], skip_nans=True):
             #   point_list.append([data[0],data[1],data[2],data[3]])
            frame_id = msg.header.frame_id
            cloud_points = np.array(list(point_cloud2.read_points(msg, 
                                                                field_names=("x", "y", "z", "intensity"), 
                                                                skip_nans=True)))
      
            cloud_points = np.array(cloud_points)
            # 保存点云为PCD文件，包括强度字段
            # 构造自定义PCD格式
            with open(f"current_{frame_id}.pcd", "w") as f:
                f.write("VERSION .7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write(f"WIDTH {cloud_points.shape[0]}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {cloud_points.shape[0]}\n")
                f.write("DATA ascii\n")
                for point in cloud_points:
                    f.write(f"{point[0]} {point[1]} {point[2]} {point[3]}\n")
            
            # 设置保存标记为True
            self.saved = True
            self.data = cloud_points
            
            # 日志信息：点云已保存
            self.get_logger().info('Point cloud saved as current_frame.pcd with intensity.')
        except Exception as e:
            # 如果保存失败，记录错误信息
            self.get_logger().error(f'Failed to save point cloud: {e}')
        finally:
            # 不论成功与否，设置事件，通知主线程点云处理完成
            self.save_event.set()
 
 
# 定义一个函数，用于启动订阅者节点并处理点云数据
def save_point_cloud(topic):
    # 初始化ROS 2环境
    rclpy.init()
        
    # 创建点云订阅者节点
    point_cloud_subscriber = PointCloudSubscriber(topic)
    
    # 创建一个单线程执行器，用于处理节点的通信
    executor = rclpy.executors.SingleThreadedExecutor()
    
    # 将节点添加到执行器中
    executor.add_node(point_cloud_subscriber)
    
    # 在一个新线程中启动执行器，开始监听和处理点云数据
    thread = threading.Thread(target=executor.spin)
    thread.start()

    
    # 等待点云数据被保存完成的事件，最多等待3秒
    if point_cloud_subscriber.save_event.wait(timeout=3):
        if point_cloud_subscriber.saved:
            # 如果点云数据保存成功，输出确认信息
            print('Point cloud saved successfully.')    
            print(point_cloud_subscriber.data)
        else:
            # 如果点云数据保存失败，输出错误信息
            print('Failed to save point cloud.')
    else:
        # 如果超过3秒还未接收到保存完成的信号，输出超时信息
        print('Saving point cloud timed out.')
    
    # 清理资源，关闭执行器
    executor.shutdown()
    
    # 销毁节点
    point_cloud_subscriber.destroy_node()
    
    # 关闭ROS 2环境
    rclpy.shutdown()
    
    # 等待线程结束
    thread.join()
 
 
 
 
# 主函数入口
def main():
    # 指定要订阅的点云话题
    save_point_cloud("/laser3d/pointcloud")
