import sys
import time
import math
import hiwonder.Mpu6050 as Mpu6050
import hiwonder.ActionGroupControl as AGC
import cv2
import threading
import numpy as np

import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.apriltag as apriltag
import hiwonder.yaml_handle as yaml_handle

mpu = Mpu6050.mpu6050(0x68)#启动Mpu6050
mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)#设置Mpu6050的陀螺仪的工作范围
mpu.set_accel_range(mpu.ACCEL_RANGE_2G)#设置Mpu6050的加速度计的工作范围

count1 = 0
count2 = 0

def standup():
    global count1, count2 

    
    try:
        accel_date = mpu.get_accel_data(g=True) #获取传感器值
        angle_y = int(math.degrees(math.atan2(accel_date['y'], accel_date['z']))) #将获得的数据转化为角度值
        
        if abs(angle_y) > 160: #y轴角度大于160，count1加1，否则清零
            count1 += 1
        else:
            count1 = 0

        if abs(angle_y) < 10: #y轴角度小于10，count2加1，否则清零
            count2 += 1
        else:
            count2 = 0

        time.sleep(0.1)
        
        if count1 >= 5: #往前倒了一定时间后起来
            count1 = 0  
            print("stand up back！")#打印执行的动作名
            AGC.runActionGroup('stand_up_back')#执行动作
        
        elif count2 >= 5: #往后倒了一定时间后起来
            count2 = 0
            print("stand up front！")#打印执行的动作名
            AGC.runActionGroup('stand_up_front')#执行动作            
        
    except BaseException as e:
        print(e)

debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lab_data = None
servo_data = None

def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)

tag_id = None
__isRunning = False
action_finish = True
# 变量重置
def reset():      
    global tag_id
    global action_finish
    
    tag_id = 0
    action_finish = True
    
# app初始化调用
def init():
    print("Apriltag Init")
    load_config()
    initMove()

# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Apriltag Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("Apriltag Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("Apriltag Exit")

def move():
    global tag_id
    global action_finish  
    
    while True:
        if debug:
            return
        if __isRunning:
            if tag_id is not None:
                action_finish = False
                time.sleep(0.0)
                if tag_id == 1:#标签ID为1时                
                    i=0
                    AGC.runActionGroup('go_forward_end')#完成最最后一步
                    while 1:
                        AGC.runActionGroup('left_move_30')#左转
                        i=i+1
                    tag_id = None
                    time.sleep(0)                  
                    action_finish = True                
                elif tag_id == 2: 
                    for i in range(40):                 
                        AGC.runActionGroup('turn_right_fast')#右转
                    tag_id = None
                    time.sleep(0)
                    action_finish = True          
                elif tag_id == 3:    
                    for i in range(40):               
                        AGC.runActionGroup('turn_left_fast')#扭腰
                    tag_id = None
                    time.sleep(1)
                    action_finish = True
                else:
                    action_finish = True
                    time.sleep(0.01)
            else:
               action_finish = False
               AGC.runActionGroup('go_forward_fast')#如果没有标志则直走
               action_finish = True
               time.sleep(0.5)
        else:
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 检测apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):   
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = int(detection.tag_id)  # 获取tag_id

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角
            
            return tag_family, tag_id, object_center_x
            
    return None, None, None

def run(img):
    global tag_id
    global action_finish
     
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    img = img[int(0.6*img_h):int(1*img_h),:]

    if not __isRunning:
        return img
    
    tag_family, tag_id, object_center_x = apriltagDetect(img) # apriltag检测

    
    if tag_id is not None:
        while object_center_x is not None:
            if object_center_x - 0.5 * img_w > 10:
                AGC.runActionGroup('turn_left_small')#扭腰
                _, _, object_center_x = apriltagDetect(img)
            elif object_center_x - 0.5 * img_w < -10:
                AGC.runActionGroup('turn_right_small')#扭腰
                _, _, object_center_x = apriltagDetect(img)
            else:
                break
        cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    else:
        cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    
    return img

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    debug = False
    if debug:
        print('Debug Mode')
        
    init()
    start()
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()        
    AGC.runActionGroup('stand')
    while True:
        ret, img = my_camera.read()
        if ret:
            standup()
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
