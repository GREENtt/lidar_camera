#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as point_cloud2
import numpy as np
import open3d as o3d
import cv2,time

class LaserListener(Node):
    def __init__(self):
        super().__init__('laser_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/laser3d/pointcloud',
            self.listener_callback,
            10
        )
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg):
        # 在这里处理激光雷达数据

        #=======param====================================================
        #雷达
        #旋转矩阵
        R_mat = np.array([
            [-0.226774620640799, -0.973860462400855, 0.0130027383890741],
            [-0.0349200362426561, -0.00521188985132232, -0.999376519272386],
            [0.973321048011412,  -0.227087287131117, -0.0328253182984725]])
	# 旋转矩阵转换为旋转向量
        rvec,_ = cv2.Rodrigues(R_mat)
	#平移矩阵 
        tvec = np.float64([0.0151114269851357, -0.0499186636061799, -0.0577138697906218])
	
         
	#相机
	#内参矩阵
        camera_matrix = np.float64([
	    [724.907550913901, 0, 1006.87260512134],
	    [0, 733.889097286267, 635.407247133659],
	    [0, 0, 1]])

	#畸变系数
        distCoeffs = np.float64(
	    [0.113098971674672, -0.180364771001407,  # 径向k1,k2,
	     0.000169560377490072, 0.000629026115977085, #切向p1,p2
	     0.158470352411919])  # 径向k3
        #=================================================================================
        
        point_list = []
        for data in point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
            point_list.append([data[0],data[1],data[2]])
        pcdarr = np.array(point_list)
        
        distance = []
        distance_3d = []
        points_3d = []
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
        cube = np.float64(points_3d) 
        print(cube.shape, len(point_list),len(distance_3d))
        point_2d, _ = cv2.projectPoints(cube,rvec,tvec,camera_matrix,distCoeffs)        
        
        cap = cv2.VideoCapture(1)          
        #v=cv2.VideoWriter(f'./vis_img2.avi', cv2.VideoWriter_fourcc(*'MJPG'),30,(720,320))
        initime = time.time()
        num = 0  
        while(cap.isOpened()):
            num += 1
            start = time.time()
            ret, frame = cap.read()        
            #print('frame.shape:',frame.shape)
            if not ret:
                break
            
            img = frame.copy()
            
            for i, point in enumerate(point_2d):
                x,y = point.ravel()
                x,y = int(x),int(y)
                if 0 <= x <= img.shape[1] and 0 <= y <= img.shape[0]:
                    #x.append(x_2d)
                    #y.append(y_2d)
                    #distance.append(distance_3d[m]*100)
                    cv2.circle(img, (x,y), 2, color=[255,255,255], thickness = -1)
            
            stop = time.time()
            fps = round(1/(stop-start), 2)  
            print("fps:",fps) 
            cv2.namedWindow('camera', cv2.WINDOW_NORMAL)
            cv2.imshow("camera", img)
            cv2.waitKey(1)        
  
        
        #points3d.points = o3d.utility.Vector3dVector(cube)
        '''
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        points3d = o3d.geometry.PointCloud()
        vis.add_geometry(points3d)
        vis.update_geometry(points3d)
        '''
        #o3d.visualization.draw_geometries([points3d])
        #self.get_logger().info("Received %d laser scan readings", cloud_points)

        


def main():
    rclpy.init()
            
    laser_listener = LaserListener()

    rclpy.spin(laser_listener)
    
    laser_listener.destroy_node() 
    rclpy.shutdown()



