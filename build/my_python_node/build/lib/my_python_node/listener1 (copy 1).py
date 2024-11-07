#!/usr/bin/env python

import rclpy
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
from tf2_ros import TransformBroadcaster, TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as point_cloud2
import numpy as np
import open3d as o3d
import cv2,time,threading
from datetime import datetime


# =======param====================================================================
#---雷达---------------------------------------------------------------------------
# 旋转矩阵
R_mat = np.array([
    [-0.226774620640799, -0.973860462400855, 0.0130027383890741],
    [-0.0349200362426561, -0.00521188985132232, -0.999376519272386],
    [0.973321048011412,  -0.227087287131117, -0.0328253182984725]])
rvec,_ = cv2.Rodrigues(R_mat) # 旋转矩阵转换为旋转向量
# 平移矩阵 
tvec = np.float64([0.0151114269851357, -0.0499186636061799, -0.0577138697906218])	
 
 
#---相机---------------------------------------------------------------------------
# 内参矩阵
camera_matrix = np.float64([
    [724.907550913901, 0, 1006.87260512134],
    [0, 733.889097286267, 635.407247133659],
    [0, 0, 1]])	    
# 畸变系数
distCoeffs = np.float64(
    [0.113098971674672, -0.180364771001407,  # 径向k1,k2,
     0.000169560377490072, 0.000629026115977085, #切向p1,p2
     0.158470352411919])  # 径向k3	     
# =================================================================================
 
out = cv2.VideoWriter(f'./lidar_img.avi', cv2.VideoWriter_fourcc(*'MJPG'), 2, (1920, 1080)) 
  
         
def get_color(cur_dep, max_dep, min_dep):
    scale = (max_dep - min_dep) / 10   #0.4999565124511719
    if cur_dep < min_dep:
        return (0, 0, 255)  # 返回蓝色
    elif cur_dep < min_dep + scale:
        green = int((cur_dep - min_dep) / scale * 255)
        return (0, green, 255)  # 返回蓝到黄的渐变色
    elif cur_dep < min_dep + scale * 2:
        red = int((cur_dep - min_dep - scale) / scale * 255)
        return (0, 255, 255 - red)  # 返回黄到红的渐变色
    elif cur_dep < min_dep + scale * 4:
        blue = int((cur_dep - min_dep - scale * 2) / scale * 255)
        return (blue, 255, 0)  # 返回红到绿的渐变色
    elif cur_dep < min_dep + scale * 7:
        green = int((cur_dep - min_dep - scale * 4) / scale * 255)
        return (255, 255 - green, 0)  # 返回绿到黄的渐变色
    elif cur_dep < min_dep + scale * 10:
        blue = int((cur_dep - min_dep - scale * 7) / scale * 255)
        return (255, 0, blue)  # 返回黄到蓝的渐变色
    else:
        return (255, 0, 255)  # 返回紫色 
        
        
# 雷达/相机数据处理节点
class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_camera')
        self.lock = threading.Lock()
        self.broadcaster = TransformBroadcaster(self)
        self.subscription_lidar = self.create_subscription(PointCloud2, '/laser3d/pointcloud',self.lidar_callback,10)   
        self.subscription_camera = self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        
        self.i=0
        self.point_2d = []
        self.distance_3d = []
        self.min_dep = 0
        self.max_dep = 0 
        self.img = None
        self.camera_time = 0
        self.scan_time = 0
        self.last_lidar_time = None
        self.last_camera_time = None  
        # 创建一个事件对象，用于线程间的同步
        #self.thread_fuse = threading.Thread(target=self.fuse)
        #self.thread_fuse.start()

        
    def lidar_callback(self, msg):
        # 在这里处理激光雷达数据
        with self.lock:
            self.last_lidar_time = msg.header.stamp

            point_list = []
            cloud_points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
            pcdarr = np.array(cloud_points)
            
            distance = []
            distance_3d = []
            points_3d = []
            min_dep = 0
            max_dep = 0
            for i, point in enumerate(pcdarr):
                x_raw = float(point[0])
                y_raw = float(point[1])
                z_raw = float(point[2])
                point_3d = []
                point_3d.append(x_raw)
                point_3d.append(y_raw)
                point_3d.append(z_raw)
                if x_raw >0:
                    points_3d.append(point_3d)
                    distance_3d.append(x_raw)  
                    if point_3d[0] > max_dep:
                        max_dep = point_3d[0]
                    if point_3d[0] < min_dep:
                        min_dep = point_3d[0]
            cube = np.float64(points_3d)  
            self.point_2d, _ = cv2.projectPoints(cube,rvec,tvec,camera_matrix,distCoeffs) 
            self.distance_3d = distance_3d 
            self.min_dep = min_dep
            self.max_dep = max_dep
            
            self.get_logger().info('11111111111111111111 Received Lidar data 11111111111111111111')
            self.lidar_msg = msg
            self.scan_time = time.time()
            #self.fuse()          
            self.synchronize()
            
    def camera_callback(self, msg):
        # 处理相机数据的代码
        with self.lock:
            initime = time.time()
            self.last_camera_time = msg.header.stamp
            # 处理图像
            bridge = CvBridge()   # 转换为ros2的消息类型(imgmsg)的工具
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8") # ros2消息类型(imgmsg)转换为np.array
            self.img = cv2.resize(cv_img, (1920, 1080))    
            
            self.get_logger().info('22222222222222222222 Received Camera data 22222222222222222222')
            self.camera_msg = msg
            self.camera_time = time.time()
            #self.fuse()
            self.synchronize()
'''
            #======激光雷达数据============================================================ 
            point_2d = self.point_2d
            distance_3d = self.distance_3d
            max_dep = self.max_dep
            min_dep = self.min_dep
            self.i = self.i + 1
            img = self.img
            #==================================================================
            if hasattr(self, 'scan_time') and hasattr(self, 'camera_time'):
                if abs(self.scan_time - self.camera_time) < 1.0:
                    self.get_logger().info("=======================success============================")
                    for i, point in enumerate(point_2d):
                        x,y = point.ravel()
                        x,y = int(x),int(y)
                        if 0 <= x <= img.shape[1] and 0 <= y <= img.shape[0]:
                            cur_depth = distance_3d[i]
                            color = get_color(cur_depth, max_dep, min_dep)
                            cv2.circle(img, (x,y), 2, color=color, thickness = -1)
             
                    stop = time.time()
                    fps = round(1/(stop-initime), 2)  
                    print("fps:",fps) 

                    cv2.putText(img, f'FPS: {str(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            print("time:",self.scan_time - self.camera_time) 
            #cv2.imshow("camera", img) # 显示接受到的图像数据
            #cv2.imwrite('./imgs' +'/' + str(self.i) + ".jpg", img)#将拍摄到的图片保存在imgs文件夹中
            #cv2.waitKey(1)   
            global out
            out.write(img)
'''
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
        self.fuse()
             
    def fuse(self):
        global out
        self.get_logger().info("======================= success ============================")
        #======激光雷达数据============================================================ 
        point_2d = self.point_2d
        distance_3d = self.distance_3d
        max_dep = self.max_dep
        min_dep = self.min_dep
        initime = time.time()
        img = self.img
        self.i = self.i + 1
        print(img.shape)
        #==================================================================        
        while rclpy.ok():
            try:
                with self.lock:
                    for i, point in enumerate(point_2d):
                        x,y = point.ravel()
                        x,y = int(x),int(y)
                        if 0 <= x <= img.shape[1] and 0 <= y <= img.shape[0]:
                            cur_depth = distance_3d[i]
                            color = get_color(cur_depth, max_dep, min_dep)
                            cv2.circle(img, (x,y), 2, color=color, thickness = -1)
            
                    stop = time.time()
                    fps = round(1/(stop-initime), 2)  
                    print("fps:",fps) 
                    cv2.putText(img, f'FPS: {str(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.imshow("camera", img)
                    cv2.imwrite('./imgs' +'/' + str(self.i) + ".jpg", img) # 将拍摄到的图片保存在imgs文件夹中
                    cv2.waitKey(1)   
                    out.write(img) 
                    self.lidar_msg = None
                    self.camera_msg = None
            except Exception as e:
                # 如果保存失败，记录错误信息
                self.get_logger().error(f'Failed to save point cloud: {e}')
             
    def destroy_node(self):
        self.thread_fuse.join()
        super().destroy_node()   
'''
def fuse(nnn):
    save_event = threading.Event()
    while rclpy.ok():
        try:
            with self.lock:
                if self.lidar_msg is not None and self.camera_msg is not None:
                    self.get_logger().info("==========1111=============Lidar Camera success============1111================")
                    #======激光雷达数据============================================================ 
                    point_2d = nnn.point_2d
                    distance_3d = nnn.distance_3d
                    max_dep = nnn.max_dep
                    min_dep = nnn.min_dep
                    initime = time.time()
                    img = nnn.img
                    nnn.i = nnn.i + 1	
		    #==================================================================
		    
                    for i, point in enumerate(point_2d):
                        x,y = point.ravel()
                        x,y = int(x),int(y)
                        if 0 <= x <= img.shape[1] and 0 <= y <= img.shape[0]:
                            cur_depth = distance_3d[i]
                            color = get_color(cur_depth, max_dep, min_dep)
                            cv2.circle(img, (x,y), 2, color=color, thickness = -1)
            
                    stop = time.time()
                    fps = round(1/(stop-initime), 2)  
                    print("fps:",fps) 
                    cv2.putText(img, f'FPS: {str(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.imshow("camera", img)
                    cv2.imwrite('./imgs' +'/' + str(nnn.i) + ".jpg", img) # 将拍摄到的图片保存在imgs文件夹中
                    cv2.waitKey(1)   
                    out.write(img) 
                    self.laser_scan_msg = None
                    self.camera_msg = None
        except Exception as e:
            # 如果保存失败，记录错误信息
            get_logger().error(f'Failed to save point cloud: {e}')
        finally:
            # 不论成功与否，设置事件，通知主线程点云处理完成
           save_event.set()
'''
 
def main():
    rclpy.init()
 
    lidar_node = LidarNode()

    
    # 创建线程用于接收雷达/相机数据
    executor = SingleThreadedExecutor()
    executor.add_node(lidar_node) 
 
    # 创建一个线程用于处理数据

    #thread = threading.Thread(target=fuse,args=(lidar_node)) 
    #thread.start()
    
    # 执行多线程执行器
    executor.spin()


    # 清理并退出
    lidar_node.destroy_node()
    executor.shutdown()


    rclpy.shutdown()

  
