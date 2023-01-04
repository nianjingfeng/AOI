import cv2 
import numpy as np
import time

from time import sleep
#import math
#import threading
import DobotDllType as dType
from pygame import mixer

from PIL import Image, ImageDraw, ImageFont
#Vision init
mask=None
capture=None
lastIndex = 5


X_Center = 300   #300
Y_Center = 260   #270
Video_num = 0
Left_Cut = 200
Right_Cut = 380
Gamma_Value = 0.4
with open('data1.txt', 'r') as file:
    data = file.read().replace('\n', '')
    pp = list(data)
    xdata = pp[2] + pp[3] + pp[4]
    ydata = pp[7] + pp[8] + pp[9]
    
    leftdata = pp[11] + pp[12] + pp[13]
    rightdata = pp[14] + pp[15] + pp[16]
    gadata = '0.' + pp[17]
    if(pp[18] == 0):
        blue_low = pp[19]+pp[20]
    else:
        blue_low = pp[18]+pp[19]+pp[20]
    if(pp[21] == 0):
        blue_max = pp[22]+pp[23]
    else:
        blue_max = pp[21]+pp[22]+pp[23]
    if(pp[24] == 0):
        green_low = pp[25]+pp[26]
    else:
        green_low = pp[24]+pp[25]+pp[26]
    if(pp[27] == 0):
        green_max = pp[28]+pp[29]
    else:
        green_max = pp[27]+pp[28]+pp[29]
    if(pp[30] == 0):
        yellow_low = pp[31]+pp[32]
    else:
        yellow_low = pp[30]+pp[31]+pp[32]
    if(pp[33] == 0):
        yellow_max = pp[34]+pp[35]
    else:
        yellow_max = pp[33]+pp[34]+pp[35]
    if(pp[36] == 0):
        suck_z = pp[37]
    else:
        suck_z = pp[36]+pp[37]



    if(pp[38] == 0):
        goal_x1 = pp[39]+pp[40]
    else:
        goal_x1 = pp[38]+pp[39]+pp[40]
    if(pp[41] == 0):
        goal_y1 = pp[42]+pp[43]
    else:
        goal_y1 = pp[41]+pp[42]+pp[43]
        
    if(pp[44] == 0):
        goal_x2 = pp[45]+pp[46]
    else:
        goal_x2 = pp[44]+pp[45]+pp[46]
    if(pp[47] == 0):
        goal_y2 = pp[48]+pp[49]
    else:
        goal_y2 = pp[47]+pp[48]+pp[49]
        
    if(pp[50] == 0):
        goal_x3 = pp[51]+pp[52]
    else:
        goal_x3 = pp[50]+pp[51]+pp[52]
    if(pp[53] == 0):
        goal_y3 = pp[54]+pp[55]
    else:
        goal_y3 = pp[53]+pp[54]+pp[55]

    Video_num = int(pp[10])
    X_Center = int(xdata)
    Y_Center = int(ydata)
    
    Left_Cut = int(leftdata)
    Right_Cut = int(rightdata)
    Gamma_Value = float(gadata)
    
    blue_low2 = int(blue_low)
    blue_max2 = int(blue_max)
    green_low2 = int(green_low)
    green_max2 = int(green_max)
    yellow_low2 = int(yellow_low)
    yellow_max2 = int(yellow_max)
    
    suck_z2 = float(suck_z)

    goal_x1 = float(goal_x1)
    goal_y1 = float(goal_y1)
    goal_x2 = float(goal_x2)
    goal_y2 = float(goal_y2)
    goal_x3 = float(goal_x3)
    goal_y3 = float(goal_y3)
"""
<--負--Y--正-->

^
| 正
X
| 負
V

"""
n1 = 0
color_th = 1500
color_state = "None"
state = "None"
kernel = np.ones((15, 15), np.uint8) 
capture = cv2.VideoCapture(Video_num)
#mask = capture

#Color_Ranger

Blue_lower = np.array([blue_low2,43,46])
Blue_upper = np.array([blue_max2,255,255])

Green_lower = np.array([green_low2,43,46])
Green_upper = np.array([green_max2,255,255])

Yellow_lower = np.array([yellow_low2,43,46])
Yellow_upper = np.array([yellow_max2,255,255])

Red_lower1 = np.array([0,43,46])
Red_upper1 = np.array([20,255,255])
Red_lower2 = np.array([156,43,46])
Red_upper2 = np.array([180,255,255])

#Dobot init
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

def cv2ImgAddText(img, text, left, top, textColor=(0, 255, 0), textSize=20):
    if (isinstance(img, np.ndarray)):  # 判断是否OpenCV图片类型
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # 创建一个可以在给定图像上绘图的对象
    draw = ImageDraw.Draw(img)
    # 字体的格式
    fontStyle = ImageFont.truetype(
        "font/simsun.ttc", textSize, encoding="utf-8")
    # 绘制文本
    draw.text((left, top), text, textColor, font=fontStyle)
    # 转换回OpenCV格式
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

def speak(file_name):
#    with tempfile.NamedTemporaryFile(delete=True) as fp:
#        tts=gTTS(text=sentence, lang=lang)
#        tts.save('{}.mp3'.format(fp.name))
        mixer.init()
        mixer.music.load(str(file_name) + '.mp3')
        mixer.music.play()

def adjust_gamma(image, gamma=1.0):

   invGamma = 1.0 / gamma
   table = np.array([((i / 255.0) ** invGamma) * 255
      for i in np.arange(0, 256)]).astype("uint8")

   return cv2.LUT(image, table)

def work(lastIndex):
    #Start to Execute Command Queued
    dType.SetQueuedCmdStartExec(api)    
    #Wait for Executing Last Command 
    while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:
        
        dType.dSleep(100)
    dType.SetQueuedCmdClear(api)
    
def Dobot_adjust():
    dType.SetEMotor(api, 0, 1, 5000,1)
    dType.SetWAITCmd(api, 12.12, isQueued=1)
    dType.SetEMotor(api, 0, 1, 0,1)
    lastIndex = dType.SetWAITCmd(api, 5, isQueued=1)
    work(lastIndex)
    
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 268.3032, 0 , 50, 0, 1)
    dType.SetWAITCmd(api, 1, isQueued=1)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 268.3032, 0 , 15, 0, 1)
    work(lastIndex)
    
    
def Dobot_work(offx, offy):
    
    dType.SetEMotor(api, 0, 1, 12500,1)    #10000
    dType.SetWAITCmd(api, 4850, isQueued=1)    #6.06
    dType.SetEMotor(api, 0, 1, 0,1)
    dType.SetWAITCmd(api, 100, isQueued=1)

    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, offx, offy , 50, 0, 1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, offx, offy , suck_z2, 0, 1)
    dType.SetEndEffectorSuctionCup(api, 1,  1, isQueued=1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, offx, offy , 70, 0, 1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)

    print("color_state = " + str(color_state))
    if(color_state == "Yellow"):
       goal_x=goal_x1
       goal_y=goal_y1
    elif(color_state == "Blue"):
       goal_x=goal_x2
       goal_y=goal_y2
    elif(color_state == "Red"):
       goal_x=goal_x3
       goal_y=goal_y3
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, goal_x, -goal_y , 70, 0, 1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, goal_x, -goal_y , 40, 0, 1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)
    dType.SetEndEffectorSuctionCup(api, 1,  0, isQueued=1)
    #手爪控制函數說明
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, goal_x, -goal_y , 70, 0, 1)
    #dType.SetWAITCmd(api, 0.1, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 270, 0 , 50, 0, 1)
    lastIndex = dType.SetWAITCmd(api, 100, isQueued=1)
    work(lastIndex)
    #sleep(0.1)
    print("End")
    
#main start
if (state == dType.DobotConnect.DobotConnect_NoError):

    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    #dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
    dType.SetPTPJointParams(api,200,200,200,200,200,200,200,200, isQueued = 1)
    dType.SetPTPCoordinateParams(api,200,200,200,200, isQueued = 1)
    #dType.SetPTPJumpParams(api, 10, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
    
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    lastIndex = dType.SetWAITCmd(api, 2000, isQueued=1)
    work(lastIndex)
    #sleep(0.5)
    
flag_adjust = False
flag_start_work = False
flag_debug = False
while(True):
    #Vision
    ret, cap_input = capture.read()
    cap2 = cap_input.copy()
    mask2 = np.zeros(cap2.shape, np.uint8)
    
    mask2[0:480, Left_Cut:Right_Cut] = 255
    
    cap_cut = cv2.bitwise_and(cap2, mask2, cap2)
    cap_blur = cv2.GaussianBlur(cap_cut , (5,5), 0)
   
    gamma = Gamma_Value                                   # change the value here to get different result
    cap_gamma = adjust_gamma(cap_blur, gamma=gamma)

    hsv = cv2.cvtColor(cap_gamma,cv2.COLOR_BGR2HSV)
    
    #hsv = cv2.erode(hsv,kernel,iterations=2)
    #hsv = cv2.dilate(hsv,kernel,iterations=2)

    Blue_mask = cv2.inRange(hsv, Blue_lower, Blue_upper)
    Green_mask = cv2.inRange(hsv, Green_lower, Green_upper)
    Yellow_mask = cv2.inRange(hsv, Yellow_lower, Yellow_upper)
    Red_mask1 = cv2.inRange(hsv, Red_lower1, Red_upper1)
    Red_mask2 = cv2.inRange(hsv, Red_lower2, Red_upper2)
    Red_mask = Red_mask1 + Red_mask2

    mask = Yellow_mask + Blue_mask + Red_mask 
    
    mask = cv2.erode(mask,kernel,iterations=1)
    mask = cv2.dilate(mask,kernel,iterations=1)
    
    c1 = cv2.countNonZero(Blue_mask)
    c2 = cv2.countNonZero(Yellow_mask)
    c3 = cv2.countNonZero(Green_mask)
    c4 = cv2.countNonZero(Red_mask)

    if (flag_start_work == True):
        n1=n1+1      
        if(True):
            n1=0
            if((c1 > c2) &(c1 > c4)):
                color_state = "Blue"
                speak(11)
                time.sleep(1)
                #mask = Blue_mask                                              
            elif((c2 > c1) &(c2 > c4)):
                color_state = "Yellow"
                speak(12)
                time.sleep(1)
                #mask = Yellow_mask
            elif((c4 > c1) &(c4 > c2)):
                #color_state = "Green"
                color_state = "Red"
                speak(14)
                #speak(13)
                time.sleep(1)
                #mask = Red_mask
                #mask = Green_mask
            
            im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            if(len(contours) > 0):
                contour = max(contours, key = cv2.contourArea)
                M = cv2.moments(contour)
                (x, y, w, h) = cv2.boundingRect(contour)
                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if(cY-Y_Center) >= 0 :
                    offy = (cY-Y_Center)*0.5001383    #0.4857797
                else:
                    offy = (cY-Y_Center)*0.5043755    #0.5053568

                if(cX-X_Center) >= 0:
                    offx = (X_Center-cX)*0.4921233      #0.52809
                else:
                    offx = (X_Center-cX)*0.5138767    #0.6079283
                offxa = 268.3032+offx
                offya = offy
                print("cX = " + str(cX))
                print("cY = " + str(cY))
                #(相機X,相機Y,類別,Z軸高度)
                Dobot_work(offxa,offya)
                flag_start_work = False
                n1=0
                color_state = "None"           
    else:
        n1=0
        color_state = "None"        
    
    if (flag_adjust == True):
        Dobot_adjust()
        flag_adjust = False

    cap_input = cv2ImgAddText(cap_input, "請將方塊置入紅色方框內", 10, 10, (255, 0 , 0), 40)
    #cv2.putText(cap_input, "請將方塊置入紅色方框內", (0, 60), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255))
    #cv2.putText(cap_input, "Y_Center = " + str(Y_Center), (30, 120), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255))
    #cv2.circle(cap_input,(X_Center, Y_Center), 3, (0, 255, 0), -1)
    cv2.rectangle(cap_input,(240,210),(400,360),(0,0,255),4)
    cv2.imshow("camera_input",cap_input)
    if (flag_debug == True):
        cv2.imshow("camera_cut",cap_cut)
        cv2.imshow("camera" ,cap_gamma)    
        cv2.imshow("camera_mask" ,mask) 
    
    keypress = cv2.waitKey(1)
    if keypress == ord('q'):
        # ESC pressed
        print("Escape hit, closing...")
        cv2.destroyAllWindows()
        break
    elif keypress == ord('g'):
        flag_start_work = True
        print("GO Work")
    elif keypress == ord('v'):
        flag_adjust = True
        print("GO adjust")
    elif keypress == ord('L'):
        flag_debug = True
        print("GO Debug")
    elif keypress == ord('r'):
        with open('data1.txt', 'r') as file:
            data = file.read().replace('\n', '')
            pp = list(data)
            xdata = pp[2] + pp[3] + pp[4]
            ydata = pp[7] + pp[8] + pp[9]
            leftdata = pp[11] + pp[12] + pp[13]
            rightdata = pp[14] + pp[15] + pp[16]
            gadata = '0.' + pp[17]
            
            if(pp[18] == 0):
                blue_low = pp[19]+pp[20]
            else:
                blue_low = pp[18]+pp[19]+pp[20]
            if(pp[21] == 0):
                blue_max = pp[22]+pp[23]
            else:
                blue_max = pp[21]+pp[22]+pp[23]
            if(pp[24] == 0):
                green_low = pp[25]+pp[26]
            else:
                green_low = pp[24]+pp[25]+pp[26]
            if(pp[27] == 0):
                green_max = pp[28]+pp[29]
            else:
                green_max = pp[27]+pp[28]+pp[29]
            if(pp[30] == 0):
                yellow_low = pp[31]+pp[32]
            else:
                yellow_low = pp[30]+pp[31]+pp[32]
            if(pp[33] == 0):
                yellow_max = pp[34]+pp[35]
            else:
                yellow_max = pp[33]+pp[34]+pp[35]
            if(pp[36] == 0):
                suck_z = pp[37]
            else:
                suck_z = pp[36]+pp[37]

            if(pp[38] == 0):
                goal_x1 = pp[39]+pp[40]
            else:
                goal_x1 = pp[38]+pp[39]+pp[40]
            if(pp[41] == 0):
                goal_y1 = pp[42]+pp[43]
            else:
                goal_y1 = pp[41]+pp[42]+pp[43]
                
            if(pp[44] == 0):
                goal_x2 = pp[45]+pp[46]
            else:
                goal_x2 = pp[44]+pp[45]+pp[46]
            if(pp[47] == 0):
                goal_y2 = pp[48]+pp[49]
            else:
                goal_y2 = pp[47]+pp[48]+pp[49]
                
            if(pp[50] == 0):
                goal_x3 = pp[51]+pp[52]
            else:
                goal_x3 = pp[50]+pp[51]+pp[52]
            if(pp[53] == 0):
                goal_y3 = pp[54]+pp[55]
            else:
                goal_y3 = pp[53]+pp[54]+pp[55]
            
            X_Center = int(xdata)
            Y_Center = int(ydata)
            Left_Cut = int(leftdata)
            Right_Cut = int(rightdata)
            Gamma_Value = float(gadata)
            blue_low2 = int(blue_low)
            blue_max2 = int(blue_max)
            green_low2 = int(green_low)
            green_max2 = int(green_max)
            yellow_low2 = int(yellow_low)
            yellow_max2 = int(yellow_max)
            suck_z2 = float(suck_z)
            goal_x1 = float(goal_x1)
            goal_y1 = float(goal_y1)
            goal_x2 = float(goal_x2)
            goal_y2 = float(goal_y2)
            goal_x3 = float(goal_x3)
            goal_y3 = float(goal_y3)
            print("Reload OK")

            
    elif keypress == ord('s'):
        X_CenterS = input("Enter New X_Center= ")
        Y_CenterS = input("Enter New Y_Center= ")
        file1 = open("data1.txt","w") 
        L = ["x="+str(X_CenterS)+"\n","y=" + str(Y_CenterS)+"\n"]
        file1.writelines(L)
        file1.close()
        with open('data1.txt', 'r') as file:
            data = file.read().replace('\n', '')
            pp = list(data)
            xdata = pp[2] + pp[3] + pp[4]
            ydata = pp[7] + pp[8] + pp[9]
            X_Center = int(xdata)
            Y_Center = int(ydata)


    
    

#Stop to Execute Command Queued
dType.SetQueuedCmdStopExec(api)
#ser.close()
cv2.destroyAllWindows()
#Disconnect Dobot
dType.DisconnectDobot(api)
