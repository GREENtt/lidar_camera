import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
import json
import os
import threading
import time
import copy

# 全局变量
sa_json_data = []
ms_json_data = []
file_list = []

#对各全局变量加锁，主线程和子线程均会对其进行操作
lock_sa = threading.Lock() 
lock_ms = threading.Lock()   
lock_f = threading.Lock()

# 数据更新事件
sa_data_updated_event = threading.Event()

# 数据更新事件
ms_data_updated_event = threading.Event()

# 文件上传事件
upload_file_event = threading.Event()

# 构造安全告警json
def sa_json(vehicle_motion, target_num, target_coordinates, img_source):
    """
    vehicle_motion: 0表示停车，1表示可继续行驶
    target_num: 发现障碍目标数量
    target_coordinates: 多个目标空间位置坐标[[x1,y1,z1],[x2,y2,z2],...]
    img_source：当前帧图像名称
    """
    target_coordinates_list = []
    for each in target_coordinates:
        json_temp = {
            "x": each[0],
            "y": each[1],
            "z": each[2]
        }
        target_coordinates_list.append(json_temp)

    data = {
        "vehicle_motion": vehicle_motion,
        "target_num": target_num,
        "target_coordinates": target_coordinates_list,
        "img_source": img_source
    }
    return data

# 构造井盖检索json
def ms_json(search, search_num, target_coordinates, img_source):
    """
    search: 1表示检索到有井盖
    search_num: 表示检索到井盖数量
    target_coordinates: 多个表示井盖在图像中的坐标[[x1,y1,w1,h1,id,type],[x2,y2,w2,h2,id,type],...]
                        x,y,w,h：目标框
                        id：井盖编号，从0开始；
                        type：井盖状态，0正常，1破损，2缺失
    img_source：当前帧图像名称
    """
    target_coordinates_list = []
    for each in target_coordinates:
        json_temp = {
            "x": each[0],
            "y": each[1],
            "w": each[2],
            "h": each[3],
            "id": each[4],
            "type": each[5]
        }
        target_coordinates_list.append(json_temp)

    data = {
        "search": search,
        "search_num": search_num,
        "target_coordinates": target_coordinates_list,
        "img_source": img_source
    }
    return data

# post请求发送json字符串
def post_json_data(session, url, json_data):
    try:
        headers = {"Content-Type": "application/json; charset = UTF-8"}
        response = session.post(url, data = json_data, headers = headers)
        print(f"{url} POST请求响应状态码：{response.status_code}")
    except Exception as e:
        print(f"{url} POST请求失败: {e}")

# post请求发送多个文件
def upload_multiple_files(session, url, files):
    """
    files: 需上传的文件，[[file_name1, file_path1], [file_name2, file_path2],...]
    """
    files_list = []
    for each in files:
        file_name = each[0]
        file_path = each[1]
        files_list.append(('images', (file_name, open(file_path, "rb"), 'image/jpeg')))
    ## files_list: [('images', ('filename1.jpg', file1, 'image/jpeg')), ('images', ('filename2.jpg', file2, 'image/jpeg'))]
    try:
        response = session.post(url, files = files_list)
        print(f"{url} POST请求响应状态码：{response.status_code}")
    except Exception as e:
        print(f"{url} POST请求失败: {e}")

# 获取要发送的数据，全局变量json_data
def get_sa_data():
    global sa_json_data
    with lock_sa:
       sa_json = copy.deepcopy(sa_json_data)
       sa_json_data.clear()   # 传出完成后清空
    return sa_json

# 发送json数据函数
def send_sa_json(url, stop_event):
    s_sa = requests.session()
    s_sa.keep_alive = False
    s_sa.mount("http://", HTTPAdapter(max_retries=Retry(total=5)))
    while not stop_event.is_set(): #检查停止时间是否已设置
        if sa_data_updated_event.wait(timeout=1): #等待数据更新事件
            sa_data_updated_event.clear()
            data = get_sa_data() #获取当前需发送数据
            json_data = json.dumps(data)
            post_json_data(s_sa, url, json_data)

# 获取要发送的数据，全局变量json_data
def get_ms_data():
    global ms_json_data
    with lock_ms:
       ms_json = copy.deepcopy(ms_json_data)
       ms_json_data.clear()   # 传出完成后清空
    return ms_json

# 发送json数据函数
def send_ms_json(url, stop_event):
    s_ms = requests.session()
    s_ms.keep_alive = False
    s_ms.mount("http://", HTTPAdapter(max_retries=Retry(total=5)))
    while not stop_event.is_set(): #检查停止时间是否已设置
        if ms_data_updated_event.wait(timeout=1): #等待数据更新事件
            ms_data_updated_event.clear()
            data = get_ms_data() #获取当前需发送数据
            json_data = json.dumps(data)
            post_json_data(s_ms, url, json_data)

# 获取要上传的文件，全局变量json_data
def get_files():
    global file_list
    with lock_f:
        files = copy.deepcopy(file_list)
        file_list.clear()  # 传出完成后清空
    return files

def send_files(url, stop_event):
    s_f = requests.session()
    s_f.keep_alive = False
    s_f.mount("http://", HTTPAdapter(max_retries=Retry(total=5)))
    while not stop_event.is_set():
        if upload_file_event.wait(timeout=1):
            upload_file_event.clear()
            files = get_files()
            upload_multiple_files(s_f, url, files)

            

if __name__ == "__main__":
    #为方便联调，用户输入参数  192.168.99.131 30096 C:\Users\lzy06\Desktop\client\jpeg
    data = input("请依次输入服务IP、端口，及待上传文件所在路径，以空格隔开：").split()
    server_ip = data[0]
    server_port = data[1]
    file_path = data[2]

    # # 服务器IP和端口
    # server_ip = "192.168.22.226"
    # server_port = "8083"
    # # 待上传文件所在路径
    # file_path = r"E:\dq\task\20240528_感知\upload_file\source"

    #各接口资源
    url_sa = "/api/v1/security-alarm"
    url_ms = "/api/v1/manhole-search"
    url_uf = "/api/v1/upload-file"


    # 安全警告服务地址   
    url_sa_sa = "http://" + server_ip + ":" + server_port + url_sa
    # 井盖检索服务地址   
    url_ms_ms = "http://" + server_ip + ":" + server_port + url_ms
    # 文件上传服务地址
    url_uf_uf = "http://" + server_ip + ":" + server_port + url_uf
    # # 自测（本地创建一个文件服务）
    # url_uf_uf = "http://" + server_ip + ":8085" + url_uf

    stop_event = threading.Event()
    # 创建并启动线程
    # 安全警告结果发送线程
    send_sa_json_thread = threading.Thread(target=send_sa_json, args=(url_sa_sa, stop_event))
    send_sa_json_thread.start()
    # 井盖检索结果发送线程
    send_ms_json_thread = threading.Thread(target=send_ms_json, args=(url_ms_ms, stop_event))
    send_ms_json_thread.start()
    # 文件上传线程
    send_files_thread = threading.Thread(target=send_files, args=(url_uf_uf, stop_event))
    send_files_thread.start()

    # 调用识别接口，获取结果
    # 上传的文件名注意唯一性
    

    for i in range(6): # 模拟识别过程运行中
        time.sleep(0.05) #暂停50毫秒，模拟识别时间

        # 发送安全警告信息
        # 示例
        target_num = 2
        target_coordinates_sa = [[2.8, 4.9, 3.2], [5.1, 3.4, 1.8]]
        img_source1 = "test" + str(2*i+1) + ".jpg"
        
        # target_num = 0
        # target_coordinates_sa = []
        # img_source1 = "" # 可继续行驶文件名为空
        
        # 实际应用中根据情况设置条件
        if target_num > 0 :
            vehicle_motion = 0 # 0停车
            json_data = sa_json(vehicle_motion, target_num, target_coordinates_sa, img_source1)
            with lock_sa:
                sa_json_data.append(json_data)
            file1 = []
            file1.append(img_source1)
            file1.append(os.path.join(file_path, img_source1))
            with lock_f:
                file_list.append(file1)
        else:
            vehicle_motion = 1 # 1可继续行驶
            json_data = sa_json(vehicle_motion, target_num, target_coordinates_sa, img_source1)
            with lock_sa:
                sa_json_data.append(json_data)

        # 发送井盖检索信息
        # 示例 
        search_num = 2
        target_coordinates_ms = [[280, 409, 320, 270, 0, 0], [751, 834, 180, 200, 1, 0]]
        img_source2 = "test" + str(2*i+2) + ".jpg"
        
        if search_num > 0:
            search = 1
            json_data = ms_json(search, search_num, target_coordinates_ms, img_source2)
            with lock_ms:
                ms_json_data.append(json_data)
            file2 = []
            file2.append(img_source2)
            file2.append(os.path.join(file_path, img_source2))
            with lock_f:
                file_list.append(file2)
        
        with lock_sa:
            if len(sa_json_data) > 4:  # 5个批量上传，上传结束后，清空重新载入
                sa_data_updated_event.set()

        with lock_ms:
            if len(ms_json_data) > 4:  # 5个批量上传，上传结束后，清空重新载入
                ms_data_updated_event.set()

        with lock_f:
            if len(file_list) > 9:  # 10个批量上传，上传结束后，清空重新载入
                upload_file_event.set()
    
    # 识别结束后，判断是否还存在未上传的结果或文件，若存在继续上传（防止一次触发失效，使用循环）
    lock_sa.acquire()    
    while len(sa_json_data) > 0:
        lock_sa.release()
        sa_data_updated_event.set()
        time.sleep(0.1)
        lock_sa.acquire()
    lock_sa.release()

    lock_ms.acquire()    
    while len(ms_json_data) > 0:
        lock_ms.release()
        ms_data_updated_event.set()
        time.sleep(0.1)
        lock_ms.acquire()
    lock_ms.release()

    lock_f.acquire()    
    while len(file_list) > 0:
        lock_f.release()
        upload_file_event.set()
        time.sleep(0.1)
        lock_f.acquire()
    lock_f.release()

    ## 调用识别接口结束后，停止线程
    stop_event.set()
    send_sa_json_thread.join()
    send_ms_json_thread.join()
    send_files_thread.join()

    #联调显示
    print("结果及文件上传测试结束!!!")
    os.system("pause")