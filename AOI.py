# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import os
import argparse
import json
import shutil
import socket
import numpy as np
import yaml
import io
import pandas as pd
import DobotDllType as dType
from pygame import mixer
# https://www.itread01.com/content/1549749989.html

"""# 初始化"""
# Vision init
mask = None
capture = None
lastIndex = 5

# 微調吸盤中心點位置
X_Center = 330  # *需調整*
Y_Center = 240  # *需調整*

"""
<--負--Y--正-->

^
| 正
X
| 負
V

"""
 
# 影像編號
Video_num = 0  # *需調整*

# 影像切割範圍 (切除無用的視野範圍)
Left_Cut = 230
Right_Cut = 390

# 亮度調整參數0.1(暗)---0.9(亮)
Gamma_Value = 0.2  # *需調整*

"""# 使用Open CV判斷顏色的程式碼"""

# 下面為不動參數
n1 = 0
color_th = 1500
color_state = "None"
state = "None"
kernel = np.ones((15, 15), np.uint8)
capture = cv2.VideoCapture(Video_num)

# Color_Ranger (使用HSV色域空間)
Blue_lower = np.array([90, 43, 46])
Blue_upper = np.array([180, 255, 255])

Green_lower = np.array([35, 43, 46])
Green_upper = np.array([80, 255, 255])

Yellow_lower = np.array([10, 43, 46])
Yellow_upper = np.array([38, 255, 255])

Red_lower1 = np.array([0, 43, 46])
Red_upper1 = np.array([20, 255, 255])
Red_lower2 = np.array([156, 43, 46])
Red_upper2 = np.array([180, 255, 255])

"""# 初始化連接機械手臂"""
# Dobot init
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# Load Dll
api = dType.load()

# Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:", CON_STR[state])

# 佇列釋放,工作執行函數
def work(lastIndex):
    # Start to Execute Command Queued
    dType.SetQueuedCmdStartExec(api)
    # Wait for Executing Last Command
    while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:

        dType.dSleep(100)
    dType.SetQueuedCmdClear(api)

"""# 可使用的副程式"""
# mp3播放函式
def speak(file_name):
    mixer.init()
    mixer.music.load(str(file_name) + '.mp3')
    mixer.music.play()

# 調整影像亮度
def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

"""# 需要修改的副程式"""
# 將相機讀取到的X,Y值,及要判斷的tag_id和吸盤高度   輸入此函數
# 輸送帶便會將物件移至手臂下,並分類.
def Dobot_work(cX, cY, tag_id, hei_z):
    # 以X_center,Y_center為中心,計算相機座標系統及手臂座標系統轉換.
    if(cY-Y_Center) >= 0:
        offy = (cY-Y_Center)*0.5001383
    else:
        offy = (cY-Y_Center)*0.5043755

    if(cX-X_Center) >= 0:
        offx = (X_Center-cX)*0.4921233
    else:
        offx = (X_Center-cX)*0.5138767
    obj_x = 260+offx
    obj_y = offy
    print(cX, cY, obj_x, obj_y)
    # 輸送帶移動至手臂下
    dType.SetEMotor(api, 0, 1, 12500, 1)
    dType.SetWAITCmd(api, 4850, isQueued=1)
    dType.SetEMotor(api, 0, 1, 0, 1)
    dType.SetWAITCmd(api, 100, isQueued=1)

    # 手臂至影像計算後及座標轉換後obj_x,obj_y位置,吸取物件
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, obj_x, obj_y, 50, 0, 1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, obj_x, obj_y, 9.0, 0, 1)
    dType.SetEndEffectorSuctionCup(api, 1,  1, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, obj_x, obj_y, 70, 0, 1)

    # 判斷是什麼物件並給予"各個類別"放置X,Y位置.  以本案為例則是黃色藍色及紅色
    print("color_state = " + str(tag_id))
    if(tag_id == "wan"):
        goal_x = 80
        goal_y = 190
    elif(tag_id == "tong"):
        goal_x = 30
        goal_y = 190
    elif(tag_id == "tiao"):
        goal_x = -20
        goal_y = 190
    elif(tag_id == "other"):
        goal_x = -70
        goal_y = 190
    # 依類別不同,將物件放置在各個位置.
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,goal_x, -goal_y, 70, 0, 1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,goal_x, -goal_y, 40, 0, 1)
    dType.SetEndEffectorSuctionCup(api, 1,  0, isQueued=1)

    # 手爪控制函數說明
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,goal_x, -goal_y, 70, 0, 1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 270, 0, 50, 0, 1)
    lastIndex = dType.SetWAITCmd(api, 100, isQueued=1)
    work(lastIndex)  # *佇列編號*
    print("End")

"""# 主程式"""

# main start
if (state == dType.DobotConnect.DobotConnect_NoError):

    # Clean Command Queued
    dType.SetQueuedCmdClear(api)
    dType.SetPTPJointParams(api, 200, 200, 200, 200,200, 200, 200, 200, isQueued=1)
    dType.SetPTPCoordinateParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
    dType.SetHOMECmd(api, temp=0, isQueued=1)
    lastIndex = dType.SetWAITCmd(api, 2000, isQueued=1)
    work(lastIndex)  # *佇列編號*
flag_start_work = False
flag_debug = False
print(1)
while(True):
    # Vision Start
    ret, cap_input = capture.read()
    cv2.imwrite('data/test.jpg', cap_input)
    cv2.waitKey(3000)
    cap2 = cap_input.copy()
    mask2 = np.zeros(cap2.shape, np.uint8)
    mask2[0:480, Left_Cut:Right_Cut] = 255
    cap_cut = cv2.bitwise_and(cap2, mask2, cap2)
    cap_blur = cv2.GaussianBlur(cap_cut , (5,5), 0)
    gamma = Gamma_Value # change the value here to get different result
    cap_gamma = adjust_gamma(cap_blur, gamma=gamma)
    #detect the object
    os.system("darknet detector test data/obj.data data/yolo-obj.cfg backup/yolo-obj_final.weights data/test.jpg  -out data/resultjson/result.json")
    majon = []
    #get the position of object
    with open('data/resultjson/result.json', 'r') as f:
        json_array = json.load(f)
        for item in json_array:
            objects = item["objects"]
            for item2 in objects:
                name = item2['name']
                center_x = item2['relative_coordinates']['center_x']
                center_y = item2['relative_coordinates']['center_y']
                confidence = item2['confidence']
                majon.append([name, center_x, center_y, confidence])
        majon = pd.DataFrame(majon)
        majon2 = majon.copy()
        for i in majon.index:
            for j in majon.index[i:]:
                difx = abs(majon.iloc[i, 1] - majon.iloc[j, 1])
                dify = abs(majon.iloc[i, 2] - majon.iloc[j, 2])
                if difx < 0.1 and dify < 0.1:
                    if majon.iloc[i, 3] > majon.iloc[j, 3]:
                        majon2 = majon2.drop([j])
                    elif majon.iloc[i, 3] < majon.iloc[j, 3]:
                        majon2 = majon2.drop([i])
        cX = majon2.iloc[0, 1]*640
        cY = majon2.iloc[0, 2]*480
        maxmajon = majon2.iloc[:, 0][majon2.iloc[:, 3] == max(majon2.iloc[:, 3])]
        clss = maxmajon.iloc[0]
    #control the mechanical arm
    Dobot_work(cX, cY, clss, 10.4)
    cv2.imshow("camera_input", cap_input)
    keypress = cv2.waitKey(1)
    if (flag_debug == True):
        cv2.imshow("camera_cut", cap_cut)
        cv2.imshow("camera", cap_gamma)
        cv2.imshow("camera_mask", mask)

    # 定義鍵盤q, g, L的功能
    if keypress == ord('q'):
        # ESC pressed
        print("Escape hit, closing...")
        cv2.destroyAllWindows()
        break
    elif keypress == ord('g'):
        flag_start_work = True
        print("GO Work")
    elif keypress == ord('L'):
        flag_debug = True
        print("GO Debug")

"""# 結束程式"""
# Stop to Execute Command Queued
dType.SetQueuedCmdStopExec(api)
# ser.close()
cv2.destroyAllWindows()
# Disconnect Dobot
dType.DisconnectDobot(api)
