#!/usr/bin/env python

import rclpy
from rclpy.time import Time
from rclpy.clock import Clock
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
        self.last_pointcloud_header = None 
        self.last_image_header = None
        # 创建一个事件对象，用于线程间的同步
        #self.thread_fuse = threading.Thread(target=self.fuse)
        #self.thread_fuse.start()

        
    def lidar_callback(self, msg):
        # 在这里处理激光雷达数据
        self.last_pointcloud_header = msg.header.stamp.sec        
        with self.lock:
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
            self.synchronize(msg)
            
    def camera_callback(self, msg):
        # 处理相机数据的代码
        self.last_image_header = msg.header.stamp.sec
        with self.lock:
            initime = time.time()
            
            # 处理图像
            bridge = CvBridge()   # 转换为ros2的消息类型(imgmsg)的工具
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8") # ros2消息类型(imgmsg)转换为np.array
            print(cv_img.shape)
            self.img = cv2.resize(cv_img, (1920, 1080))    
            
            self.get_logger().info('22222222222222222222 Received Camera data 22222222222222222222')
            self.camera_msg = msg
            self.camera_time = time.time()
            #self.fuse()
            self.synchronize(msg)

    def synchronize(self,msg):   
        if self.last_pointcloud_header is not None and self.last_image_header is not None:
            print(self.last_pointcloud_header)
            print(self.last_image_header)
            pointcloud_time = Time(self.last_pointcloud_header)
            image_time = Time(self.last_image_header)
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
                self.fuse(msg)
                #delattr(self, 'last_pointcloud_header')
                #delattr(self, 'last_image_header')
             
    def fuse(self,msg):
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
  

    
    
    
'''
rclpy.init() 

import glob
import os
import urllib
import traceback
import time
import sys
import numpy as np
import cv2
from rknnlite.api import RKNNLite
from math import exp

RKNN_MODEL = '/home/zxq/algorithm/detect/model/detect_FQ.rknn'

dataset_file = './dataset.txt'
img_folder = "./dataset"
video_path = "/home/zxq/algorithm/detect/00001.mp4"
video_inference = True

result_path = './detect_result'
CLASSES = ['broke', 'good', 'lose']

meshgrid = []

class_num = len(CLASSES)
headNum = 3
strides = [8, 16, 32]
mapSize = [[80, 80], [40, 40], [20, 20]]
nmsThresh = 0.5
objectThresh = 0.5

input_imgH = 640
input_imgW = 640


class DetectBox:
    def __init__(self, classId, score, xmin, ymin, xmax, ymax):
        self.classId = classId
        self.score = score
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax


def GenerateMeshgrid():
    for index in range(headNum):
        for i in range(mapSize[index][0]):
            for j in range(mapSize[index][1]):
                meshgrid.append(j + 0.5)
                meshgrid.append(i + 0.5)


def IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2):
    xmin = max(xmin1, xmin2)
    ymin = max(ymin1, ymin2)
    xmax = min(xmax1, xmax2)
    ymax = min(ymax1, ymax2)

    innerWidth = xmax - xmin
    innerHeight = ymax - ymin

    innerWidth = innerWidth if innerWidth > 0 else 0
    innerHeight = innerHeight if innerHeight > 0 else 0

    innerArea = innerWidth * innerHeight

    area1 = (xmax1 - xmin1) * (ymax1 - ymin1)
    area2 = (xmax2 - xmin2) * (ymax2 - ymin2)

    total = area1 + area2 - innerArea

    return innerArea / total


def NMS(detectResult):
    predBoxs = []

    sort_detectboxs = sorted(detectResult, key=lambda x: x.score, reverse=True)

    for i in range(len(sort_detectboxs)):
        xmin1 = sort_detectboxs[i].xmin
        ymin1 = sort_detectboxs[i].ymin
        xmax1 = sort_detectboxs[i].xmax
        ymax1 = sort_detectboxs[i].ymax
        classId = sort_detectboxs[i].classId

        if sort_detectboxs[i].classId != -1:
            predBoxs.append(sort_detectboxs[i])
            for j in range(i + 1, len(sort_detectboxs), 1):
                if classId == sort_detectboxs[j].classId:
                    xmin2 = sort_detectboxs[j].xmin
                    ymin2 = sort_detectboxs[j].ymin
                    xmax2 = sort_detectboxs[j].xmax
                    ymax2 = sort_detectboxs[j].ymax
                    iou = IOU(xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2)
                    if iou > nmsThresh:
                        sort_detectboxs[j].classId = -1
    return predBoxs


def sigmoid(x):
    return 1 / (1 + exp(-x))


def postprocess(out, img_h, img_w):
    print('postprocess ... ')

    detectResult = []
    output = []
    for i in range(len(out)):
        print(out[i].shape)
        output.append(out[i].reshape((-1)))

    scale_h = img_h / input_imgH
    scale_w = img_w / input_imgW

    gridIndex = -2
    cls_index = 0
    cls_max = 0

    for index in range(headNum):
        reg = output[index * 2 + 0]
        cls = output[index * 2 + 1]

        for h in range(mapSize[index][0]):
            for w in range(mapSize[index][1]):
                gridIndex += 2

                if 1 == class_num:
                    cls_max = sigmoid(cls[0 * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w])
                    cls_index = 0
                else:
                    for cl in range(class_num):
                        cls_val = cls[cl * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][1] + w]
                        if 0 == cl:
                            cls_max = cls_val
                            cls_index = cl
                        else:
                            if cls_val > cls_max:
                                cls_max = cls_val
                                cls_index = cl
                    cls_max = sigmoid(cls_max)

                if cls_max > objectThresh:
                    regdfl = []
                    for lc in range(4):
                        sfsum = 0
                        locval = 0
                        for df in range(16):
                            temp = exp(reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h *
                                           mapSize[index][1] + w])
                            reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][
                                1] + w] = temp
                            sfsum += temp

                        for df in range(16):
                            sfval = reg[((lc * 16) + df) * mapSize[index][0] * mapSize[index][1] + h * mapSize[index][
                                1] + w] / sfsum
                            locval += sfval * df
                        regdfl.append(locval)

                    x1 = (meshgrid[gridIndex + 0] - regdfl[0]) * strides[index]
                    y1 = (meshgrid[gridIndex + 1] - regdfl[1]) * strides[index]
                    x2 = (meshgrid[gridIndex + 0] + regdfl[2]) * strides[index]
                    y2 = (meshgrid[gridIndex + 1] + regdfl[3]) * strides[index]

                    xmin = x1 * scale_w
                    ymin = y1 * scale_h
                    xmax = x2 * scale_w
                    ymax = y2 * scale_h

                    xmin = xmin if xmin > 0 else 0
                    ymin = ymin if ymin > 0 else 0
                    xmax = xmax if xmax < img_w else img_w
                    ymax = ymax if ymax < img_h else img_h

                    box = DetectBox(cls_index, cls_max, xmin, ymin, xmax, ymax)
                    detectResult.append(box)
    # NMS
    print('detectResult:', len(detectResult))
    predBox = NMS(detectResult)

    return predBox


def export_rknnlite_inference(img):
    # Create RKNN object
    rknnlite = RKNNLite(verbose=False)

    # Load ONNX model
    print('--> Loading model')
    ret = rknnlite.load_rknn(RKNN_MODEL)
    if ret != 0:
        print('Load model failed!')
        exit(ret)
    print('done')

    # Init runtime environment
    print('--> Init runtime environment')
    # ret = rknnlite.init_runtime()
    ret = rknnlite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    if ret != 0:
        print('Init runtime environment failed!')
        exit(ret)
    print('done')

    # Inference
    print('--> Running model')
    outputs = rknnlite.inference(inputs=[img])
    rknnlite.release()
    print('done')

    return outputs


def get_dataset_txt(dataset_path, dataset_savefile):
    file_data = glob.glob(os.path.join(dataset_path, "*.png"))
    with open(dataset_savefile, "r") as f:
        for file in file_data:
            f.readlines(f"{file}\n")


if __name__ == '__main__':
    print('This is main ...')
    GenerateMeshgrid()
    isExist = os.path.exists(result_path)
    if not isExist:
        os.makedirs(result_path)
    rknn_lite = RKNNLite(verbose=False)
    ret = rknn_lite.load_rknn(RKNN_MODEL)
    ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)

    if video_inference == False:
        print('--> image -----------------------------------------')
        img_names = os.listdir(img_folder)
        initime = time.time()
        num = 0
        allfps =0
        for name in img_names:
            img_path = os.path.join(img_folder, name)
            num += 1
           
            orig_img = cv2.imread(img_path)
            img_h, img_w = orig_img.shape[:2]

            origimg = cv2.resize(orig_img, (input_imgW, input_imgH), interpolation=cv2.INTER_LINEAR)
            origimg = cv2.cvtColor(origimg, cv2.COLOR_BGR2RGB)

            img = np.expand_dims(origimg, 0)
            start = time.time()
            outputs = rknn_lite.inference(inputs=[img])   # outputs = export_rknnlite_inference(img)

            out = []
            for i in range(len(outputs)):
                out.append(outputs[i])

            predbox = postprocess(out, img_h, img_w)

            print('detect:', len(predbox))
            fps = 1 / (time.time() - start)
            allfps += fps
            
            print('fps: ', fps)
            for i in range(len(predbox)):
                xmin = int(predbox[i].xmin)
                ymin = int(predbox[i].ymin)
                xmax = int(predbox[i].xmax)
                ymax = int(predbox[i].ymax)
                classId = predbox[i].classId
                score = predbox[i].score

                cv2.rectangle(orig_img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                ptext = (xmin, ymin)
                title = CLASSES[classId] + ":%.2f" % (score)
                cv2.putText(orig_img, title, ptext, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            
            cv2.imwrite(f'./{result_path}/{name}', orig_img)
            cv2.imshow("test", orig_img)
            cv2.waitKey(1)
        
        end = time.time()
        print('avgFPS, avgTime:', allfps/num, (end - initime)/num)
    else:
        print('--> video -----------------------------------------')
        cap = cv2.VideoCapture(video_path)
        initime = time.time()
        num = 0
        v = cv2.VideoWriter(f'./{result_path}/detect.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1920, 1080))
        allfps = 0
        while (cap.isOpened()):
            num += 1
            ret, frame = cap.read()
            print('ret:', ret)
            if not ret:
                break
            img_h, img_w = frame.shape[:2]

            orig_img = cv2.resize(frame, (input_imgW, input_imgH), interpolation=cv2.INTER_LINEAR)
            orig_img = cv2.cvtColor(orig_img, cv2.COLOR_BGR2RGB)

            img = np.expand_dims(orig_img, 0)
            start = time.time()
            outputs = rknn_lite.inference(inputs=[img]) #outputs = export_rknnlite_inference(img)

            out = []
            for i in range(len(outputs)):
                out.append(outputs[i])

            predbox = postprocess(out, img_h, img_w)

            print('detect:', len(predbox))
            fps = 1 / (time.time() - start)
            allfps += fps

            print('fps: ', fps)
            for i in range(len(predbox)):
                xmin = int(predbox[i].xmin)
                ymin = int(predbox[i].ymin)
                xmax = int(predbox[i].xmax)
                ymax = int(predbox[i].ymax)
                classId = predbox[i].classId
                score = predbox[i].score
                print(f'point  score :', CLASSES[classId], score)

                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                ptext = (xmin, ymin)
                title = CLASSES[classId] + ":%.2f" % (score)
                cv2.putText(frame, title, ptext, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)


            cv2.imshow("output", frame)
            # i = cv2.resize(frame, (640, 640))
            v.write(frame)
            cv2.imwrite(f'./{result_path}/test_rknn_result.jpg', frame)
            cv2.waitKey(1)
        end = time.time()
        print('avgFPS, avgTime:', allfps/num, (end - initime)/num)
    rknn_lite.release()
    '''
        

