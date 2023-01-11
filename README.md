# AOI
The project of course Practical AOI and Machine Vision
## Guide

The final report of this course is to classify objects using object recognition technology with a robotic arm, and the category of the object is selected by each group.<br>
In this final report, we choose mahjong as our classification object, and we use YOLOv4 for object recognition under Windows 10 environment. [Click here](https://github.com/Tianxiaomo/pytorch-YOLOv4) to know more information about YOLOv4. Please check AOI.py to get the code in this final report.

Setting
---
Three equipments are used in this final report:<br>
1. Server
2. Conveyor+Camera
3. Robotic arm<br>
![](https://github.com/nianjingfeng/AOI/blob/master/image/robot.jpg)

Models
---
There are 4 categories to classify:
1. WAN(萬): From number 1 to 9.![](https://github.com/nianjingfeng/AOI/blob/master/image/wan_data.jpg)
3. TONG(筒): From number 1 to 9.![](https://github.com/nianjingfeng/AOI/blob/master/image/tong_data.jpg)
4. TIAO(條): From number 1 to 9.![](https://github.com/nianjingfeng/AOI/blob/master/image/tiao_data.jpg)
5. Other: Particular pattern.![](https://github.com/nianjingfeng/AOI/blob/master/image/other_data.jpg)

Process
---
The process of this final report is shown below:
1. Place an item on conveyor.
2. The camera on the conveyor detects an item.
3. When an object is detected, the conveyor transports an item to a position where the robotic arm can pick it up.
4. The robotic arm picks an item according to the detected coordinates.
5. The robotic arm places an item in the specified position.

Inference
---
![](https://github.com/nianjingfeng/AOI/blob/master/image/inference_video.gif)
