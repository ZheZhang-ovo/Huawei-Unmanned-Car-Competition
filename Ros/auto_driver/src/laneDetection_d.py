#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#================修改内容================
#二值化阈值
#人行道距离判断

import rospy
import cv2
import os
import sys
import glob
import numpy as np
# import math
from math import sin 
from math import atan
from math import cos
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import time
import matplotlib.pyplot as plt

#距离映射
#x_cmPerPixel=53/240.00
#y_cmPerPixel = 75/360.00
x_cmPerPixel=53/(200.00/2)
y_cmPerPixel = 75/(360.00/2)
#x_cmPerPixel = 53/440.00
#y_cmPerPixel = 75/360.00
#roadWidth = 390 #454 #车道线的宽度
roadWidth = 180
y_offset = 50.0 #cm 摄像头到轮子的距离

#轴间距
I = 58.0
#摄像头坐标系与车中心间距
D = 18.0
#计算cmdSteer的系数
k = -21
k0 = -20.
#aimLine 目标行数
aim_line = 300/2
actual_speed_data = 0.




def Callback_Speed(msg):
    global actual_speed_data
    actual_speed_data = msg.data
    #print("speed")
    #rospy.loginfo(msg)


class camera:
    def __init__(self):

        self.camMat = []
        self.camDistortion = []

        # self.cap = cv2.VideoCapture('/dev/video10') #读取设备图像
        self.cap = cv2.VideoCapture('D:\\Self-Documents\\HuaWei-AutoCar\\final\\2nd_camera_test.avi') # 读取文件中的视频
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


        # self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        self.pdFlg = rospy.Publisher('pedestrians_flag', Int32, queue_size=1)
        # self.sound_pub = rospy.Publisher("/soundRequest", Int32, queue_size = 1)
       
        self.pedestrians_flag = 0
        self.cam_cmd = Twist()
        self.cvb = CvBridge()
        
        # 透视
        #src_points = np.array([[0, 352], [235, 208], [405, 208], [639, 352]], dtype="float32")
        #dst_points = np.array([[199., 360.], [199., 10.], [440., 10.], [440., 360.]], dtype="float32")
        # src_points = np.array([[0, 352*2], [235*2, 208*2], [405*2, 208*2], [639*2, 352*2]], dtype="float32")
        # dst_points = np.array([[199*2, 360*2], [199*2, 10*2], [440*2, 10.], [440*2, 360*2]], dtype="float32")
        #src_points = np.array([[0,337],[333,224],[424,224],[812,337]], dtype="float32")
        #dst_points = np.array([[100,480],[100,0],[540,0],[540,480]], dtype="float32")
        src_points = np.array([[0, 352. /2], [235. /2, 208. /2], [405. /2, 208. /2], [639. /2, 352. /2]], dtype="float32")
        dst_points = np.array([[229. /2 , 360. /2], [229. /2, 10. /2], [410. /2, 10. /2], [410. /2, 360. /2]], dtype="float32")

        self.M = cv2.getPerspectiveTransform(src_points, dst_points)
        
        #保存参数值
        self.aP = [0.0, 0.0] #目标点坐标(真实坐标&图像坐标)
        self.lastP = [0.0, 0.0] #前目标点坐标(真实坐标)
        #self.size = [640, 360]
        self.size = [320, 180]
        #self.size = [640, 480]
        self.Timer = 0 #计数 由于防跳变
        # self.pre_aim=[0,320]
        self.upper_half_histSum = 0 #上半部分总白点数
        self.lower_half_histSum = 0 #下半部分总白点数
        
        self.pre_flag = False #连续帧检测flag
        self.pre_flag_num = 0 #连续帧检测 连续拟合时间
        self.pre_x_intertcept = 0 #前x截距
        self.pre_inds = [0,0,0] #前拟合曲线方程系数
        self.lane_base = 0 #车道线起始点横坐标
        self.count = 0 #存图间隔计数
        self.img_count = 0 #存图数目
        self.pre_angle = 0 #上一刻打角角度
        self.pre_aim_x = self.size[0]/2 #上一刻目标点横坐标
        self.ped_dectcion_flg = 0#人行道检测标志

        #self.m = 0
    
    def __del__(self):
        self.cap.release()
        
    
    #======hsl二值化========
    '''
    def hlsLSelect(self, img, thresh=(170, 255)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        #imshow(l_channel)
        l_channel = l_channel * (255. / (np.max(l_channel)+1e-10))
        binary_output = np.zeros_like(l_channel)
        binary_output[(l_channel > thresh[0]) & (l_channel <= thresh[1])] = 255
        return binary_output
    '''
    
    #0920过滤斑马线
    def hlsLSelect(self, img, thresh =(20, 38, 150, 255)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        h_channel = hls[:, :, 0]
        l_channel = hls[:, :, 1]
        #imshow(l_channel)
        #l_channel = l_channel * (255. / (np.max(l_channel)+1e-10))
        binary_output = np.zeros_like(l_channel)
        binary_output[(h_channel > thresh[0]) & (h_channel <= thresh[1]) & (l_channel > thresh[2]) & (l_channel <= thresh[3])] = 255
        return binary_output
    


    '''
    def OTSU(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, binary_output = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        print("峰值为: ", ret)
        return binary_output
    '''

    """
    搜索斑马线 默认从左弯进入人行道
    """
    def find_pedestrians(self, nwindows, nonzerox, nonzeroy, minpix,midpoint_x):
        window_height = int(self.size[1] / nwindows) # 设置滑窗的高度 size[1]表示高度
        x_intertcept = self.pre_inds[0] * (self.size[1] ** 2) + self.pre_inds[1] * self.size[1] + self.pre_inds[2] # 计算拟合直线的截距
        delta_x = midpoint_x - x_intertcept # 用于判断左右车道线
        pedestrians_ins = [] # 初始化像素点位置
        if delta_x > 0:
            for window in range(nwindows): # 滑窗循环
                win_y_low = window * window_height # 滑窗上边界
                win_y_high = (window+1) * window_height # 滑窗下边界
                current_base_y = (win_y_low + win_y_high)/2 # 滑窗中心点y
                # current_base_x = aim_x = self.cal_aim_point(self.pre_inds, midpoint_x, x_intertcept, current_base_y) # 滑窗中心x
                win_x_high = min(self.pre_inds[0] * (win_y_low ** 2) + self.pre_inds[1] * win_y_low + self.pre_inds[2] - 8/x_cmPerPixel,self.size[0]) # 滑窗右边界
                # win_x_low = max(2*midpoint_x - win_x_high, 0) # 滑窗左边界
                win_x_low = 0
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                        nonzerox < win_x_high)).nonzero()[0]

                if len(good_inds) > minpix:
                    pedestrians_ins.append(good_inds)
        else:
            for window in range(nwindows): # 滑窗循环
                win_y_low = window * window_height # 滑窗上边界
                win_y_high = (window+1) * window_height # 滑窗下边界
                current_base_y = (win_y_low + win_y_high)/2 # 滑窗中心点y
                # current_base_x = aim_x = self.cal_aim_point(self.pre_inds, midpoint_x, x_intertcept, current_base_y) # 滑窗中心x

                win_x_low = max(self.pre_inds[0] * (win_y_low ** 2) + self.pre_inds[1] * win_y_low + self.pre_inds[2] + 8/x_cmPerPixel,0) # 滑窗左边界
                # win_x_high = min(2*midpoint_x - win_x_low, 1280) # 滑窗右边界
                win_x_high = self.size[0] #size[0]表示图片的宽度
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                        nonzerox < win_x_high)).nonzero()[0] #选取符合点数

                if len(good_inds) > minpix:#重新定滑窗中心
                    pedestrians_ins.append(good_inds)
        if len(pedestrians_ins):#将list整成一维数组
            pedestrians_ins = np.concatenate(pedestrians_ins)
        return pedestrians_ins

    """
    搜索临近点
    """
    def search_around_pixels(self, nwindows, nonzerox, nonzeroy, minpix, margin):
        window_height = int(self.size[1] / nwindows)
        around_pixels_ins = []
        for window in range(nwindows):
            win_y_low = window * window_height
            win_y_high = (window+1) * window_height
            current_base_y = (win_y_low + win_y_high)/2
            current_base_x = self.pre_inds[0] * (current_base_y ** 2) + self.pre_inds[1] * current_base_y + \
                             self.pre_inds[2]
            win_x_low = max(current_base_x - margin,0)
            win_x_high = min(current_base_x + margin, self.size[0])
            good_around_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                    nonzerox < win_x_high)).nonzero()[0]
            if len(good_around_inds) > minpix:
                around_pixels_ins.append(good_around_inds)
        if len(around_pixels_ins):
            around_pixels_ins = np.concatenate(around_pixels_ins)
        return around_pixels_ins

    """
    寻找车道线像素点
    """
    def find_lane_pixels(self, binary_warped, nwindows, margin, minpix):
                # start1 = time.time()
        # Take a histogram of the bottom half of the image
        histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0)#直方图 按列
        midpoint_x = int(histogram_x.shape[0] / 2) #取图像横坐标中点
        self.lane_base = np.argmax(histogram_x) #选取最大值点作为车道线起始点

        histogram_y = np.sum(binary_warped[0:binary_warped.shape[0],:], axis=1)#直方图 按行
        midpoint_y = int(histogram_y.shape[0]/3)
        self.upper_half_histSum = np.sum(histogram_y[0:midpoint_y]) #上半部分总点数
        self.lower_half_histSum = np.sum(histogram_y[midpoint_y:]) #下半部分总白点数

        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero() #取出非零像素值坐标点
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        lane_current = self.lane_base
        # end1 = time.time()
        
        #=================================人行道检测=========================
        #--------------------display-----------------------------------------
        print("===============self.upper_half_histSum",self.upper_half_histSum)
        print("===============self.lower_half_histSum",self.lower_half_histSum)
        print("===============rate",self.upper_half_histSum/(self.lower_half_histSum+1e-10))
        #--------------------------------------------------------------------
        
        # if (self.upper_half_histSum > 14500000 or self.lower_half_histSum > 14500000) and np.sum(self.pre_inds):#14500000为阈值数
        if np.sum(self.pre_inds):
            if (self.upper_half_histSum/(self.lower_half_histSum+1e-10) < 0.15 and self.lower_half_histSum > 5000000) or self.upper_half_histSum > 7600000 or self.lower_half_histSum > 7600000:#30000000
                pedestrians_inds = self.find_pedestrians(int(nwindows/2.0), nonzerox, nonzeroy, minpix, midpoint_x)
                pedestrians_x = nonzerox[pedestrians_inds]
                pedestrians_y = nonzeroy[pedestrians_inds]
                
                # 停车控制
                print("=====================len===========",len(pedestrians_x))
                if len(pedestrians_x) and len(pedestrians_y):
                    # pd_aveX =pedestrians_x[len(pedestrians_x)/2]
                    pd_aveY = np.max(pedestrians_y)
                    print(pd_aveY)
                    self.ped_dectcion_flg = 1
                    # dis = (((pd_aveX-midpoint_x)*x_cmPerPixel)**2+((720 - pd_aveY)*x_cmPerPixel+y_offset)**2)
                    # print('==================dis=================',dis)
                    # if dis < 6000:
                        # self.pedestrians_flag = 1
                        # print("===============ped_flag is True============")
                    # else:
                        # self.pedestrians_flag = 0
                    if pd_aveY > 193:
                        self.pedestrians_flag = 1
                        print("===============ped_flag is True============")
                    else:
                        self.pedestrians_flag = 0
                #=====================display====================
                # cv2.imshow('binary_warped2 ', binary_warped )
                #================================================
                else:
                    self.ped_dectcion_flg = 0
                    
                # 原图中除去人行道，重定义非零的x和y
                binary_warped[pedestrians_y, pedestrians_x]=0 #将人行道坐标点像素值置为0
                nonzero = binary_warped.nonzero() #取出非零像素值坐标点
                nonzeroy = np.array(nonzero[0])
                nonzerox = np.array(nonzero[1])
                histogram_x = np.sum(binary_warped[int(binary_warped.shape[0] / 2):, :], axis=0) #重定直方图
                self.lane_base = np.argmax(histogram_x)
                lane_current = self.lane_base
            else:
                self.pedestrians_flag = 0
                self.ped_dectcion_flg = 0
        else:
            self.pedestrians_flag = 0
            self.ped_dectcion_flg = 0
        #==============打印车道线起始点==================
        print("lane_base",self.lane_base)
        #=====================display====================
        ##cv2.imshow('binary_warped3 ', binary_warped )
        #================================================
        
        # end2 = time.time()
        # if np.sum(self.pre_inds):
            # around_pixels = self.search_around_pixels(nwindows, nonzerox, nonzeroy, minpix, margin)
            # if len(around_pixels) > self.pre_pixels_num*0.90 and self.pre_flag_num < 3:
                # lane_x = nonzerox[around_pixels]
                # lane_y = nonzeroy[around_pixels]
                # self.pre_flag = True
                # self.pre_flag_num += 1
                # return lane_x, lane_y, midpoint_x
            # else:
                # self.pre_flag = False
                # self.pre_flag_num = 0

        # plt.plot(nonzerox,nonzeroy)
        # plt.show()
        
        # 使用滑窗检索车道线
        lane_inds = []
        for window in range(nwindows):
        
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_x_low = lane_current - margin
            win_x_high = lane_current + margin
            # Identify the nonzero pixels in x and y within the window #
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_inds) > minpix:
                lane_inds.append(good_inds)
                lane_current = int(np.mean(nonzerox[good_inds]))
            
        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            if(len(lane_inds)):
                lane_inds = np.concatenate(lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass
        # end3 = time.time()
        self.pre_pixels_num = len(lane_inds) #计算车道线像素点数目
        lane_x = nonzerox[lane_inds] #取出车道线横坐标
        lane_y = nonzeroy[lane_inds] #取出车道线纵坐标
        # end4 = time.time()
        # print("time1",end1 - start1)
        # print("time2",end2 - start1)
        # print("time3",end3 - start1)
        # print("time4",end4 - start1)
        return lane_x,lane_y, midpoint_x

    """
    寻找目标点
    """
    def find_aim_point(self,lane_x, lane_y, midpoint_x):
        try:
            # id = np.argsort(lane_y)[int(len(lane_y) / 8)]
            # print(lane_y[id])
            if self.pre_flag is False:#连续帧检测中，如果前一帧拟合的曲线不符合本帧图像
                if len(lane_x) and len(lane_y):
                    _inds = np.polyfit(lane_y, lane_x, 2)#拟合曲线，以y为自变量

                if(len(_inds)):
                    x_intertcept = _inds[0] * (self.size[1] ** 2) + _inds[1] * self.size[1] + _inds[2]#计算x轴截距
                    # 
                    # if abs(x_intertcept - self.pre_x_intertcept) > 200 and self.pre_x_intertcept > 0 \
                    # and (x_intertcept - midpoint_x)*(self.pre_x_intertcept)>0 :
                        # x_intertcept = self.pre_x_intertcept
                        # _inds = self.pre_inds
                    
                    # self.pre_x_intertcept = x_intertcept#保留前一刻截距值
                    # print(_inds, midpoint_x, x_intertcept, aim_line)
                    # start1 = time.time()
                    aim_x = self.cal_aim_point(_inds, midpoint_x, x_intertcept, aim_line)#计算目标行的横坐标
                    # end1=time.time()
                    # print("cal_aim_point",end1-start1)
                    self.pre_inds = _inds#保留拟合曲线方程系数用于人行道检测
                else:
                    aim_x = midpoint_x
            else:#连续帧检测中，如果前一帧拟合的曲线符合本帧图像
                x_intertcept = self.pre_inds[0] * (self.size[1] ** 2) + self.pre_inds[1] * self.size[1] + self.pre_inds[2]
                aim_x = self.cal_aim_point(self.pre_inds, midpoint_x, x_intertcept, aim_line)
        except:
            aim_x = midpoint_x
        # if len(lane_y) > 0:
            # aim_y = lane_y[id]
        # else:
        aim_y = aim_line
        aim_x = max(min(aim_x, self.size[0]), 0)
        aim_y = max(min(aim_y, self.size[1]), 0)
        #===============人行道修正打角================
        #if self.ped_dectcion_flg == 1 and self.pre_aim_x*aim_x < 0:
        if self.pre_aim_x * aim_x < 0:
            aim_x = self.pre_aim_x
        ## 
        alpha = 0.7
        aim_x = alpha * aim_x + (1 - alpha) * self.pre_aim_x 
        self.pre_aim_x = aim_x            
        #=============================================
        
        # print("aim_x", aim_x)
        return aim_x, aim_y

    """ 
    计算目标点横坐标
    """
    def cal_aim_point(self, inds, midpoint_x, x_intertcept, aim_line_y):
        try:
            lane_Pk = 2 * inds[0] * aim_line_y + inds[1]
            # print("lane_Pk", lane_Pk)
            k_ver = - 1 / (lane_Pk+1e-10)
            theta = atan(k_ver)
            theta = max(min(theta,179),1)
            delta_x = midpoint_x - self.lane_base
            # print("1")
            # if abs(lane_Pk) < 0.1:
                # # print("delta_x",delta_x)
                # if delta_x < 0:
                    # delta_x_ = -roadWidth*0.55
                # else:
                    # delta_x_ = roadWidth*0.55
                # # print("delta_x_",delta_x_)
                # # aim_x = x_intertcept + delta_x_
            # elif abs(lane_Pk) < 3:
                # if delta_x < 0:
                    # if x_intertcept < 880:
                        # delta_x_ = -roadWidth*0.78
                    # else:
                        # delta_x_ = -roadWidth*0.58
                # else:
                    # if x_intertcept > 399:
                        # delta_x_ = roadWidth * 0.78
                    # else:
                        # delta_x_ = roadWidth*0.58
                # delta_x_ /= sin(abs(theta))
                # # aim_x = inds[0] * (aim_line ** 2) + inds[1] * aim_line + inds[2] + delta_x_
            # else:
                # if delta_x < 0:
                    # delta_x_ = -roadWidth*0.63
                # else:
                    # delta_x_ = roadWidth*0.63
            print("============lane_Pk",lane_Pk)
            if delta_x < 0:
                if abs(lane_Pk)<0.1:
                    delta_x_ = -roadWidth*0.55
                elif abs(lane_Pk)<4.5:
                    if x_intertcept < 440:
                        delta_x_ = -roadWidth*0.68
                    else:
                        delta_x_ = -roadWidth*0.64
                else:
                    delta_x_ = -roadWidth*0.55
                y = aim_line + delta_x_*cos(theta)
            else:
                if abs(lane_Pk)<0.1:
                    delta_x_ = roadWidth*0.55
                elif abs(lane_Pk)<0.4:
                    delta_x_ = roadWidth*0.6
                elif abs(lane_Pk)<4.5:
                    if x_intertcept > 199:
                        delta_x_ = roadWidth*0.68
                    else:
                        delta_x_ = roadWidth*0.64
                else:
                    delta_x_ = roadWidth*0.55
                y = aim_line + delta_x_*cos(180-theta)
            

            # x = inds[0] * (y ** 2) + inds[1] * y + inds[2] + delta_x_*sin(theta)
            delta_x_1 = inds[0]*(aim_line-y)*(aim_line+y)+inds[1]*(aim_line-y)
            aim_x = inds[0] * (aim_line ** 2) + inds[1] * aim_line + inds[2] + delta_x_*sin(theta) + delta_x_1
            
            
            # if delta_x < 0:
                # if x_intertcept < 880:
                    # delta_x_ = -roadWidth*0.85
                # else:
                    # delta_x_ = -roadWidth*0.65
            # else:
                # if x_intertcept > 399:
                    # delta_x_ = roadWidth * 0.85
                # else:
                    # delta_x_ = roadWidth*0.65
            # delta_x_ /= math.sin(abs(theta))
            # print("aimLine_line_y",aim_line_y)
            # print("inds",inds)
            aim_x = inds[0] * (aim_line_y ** 2) + inds[1] * aim_line_y + inds[2] + delta_x_
            # print("aim_x",aim_x)
            if inds[0] < 0 and lane_Pk >0 and aim_x > self.lane_base:
                aim_x = 2 * midpoint_x - aim_x
            elif inds[0] > 0 and lane_Pk < 0 and aim_x < self.lane_base:
                aim_x = 2 * midpoint_x - aim_x        
        except:
            aim_x = midpoint_x  
        # self.pre_aim[0] = aim_x
        return aim_x

    def spin(self):
        # start1 = time.time()
        ret, img = self.cap.read()
        
        # end1 = time.time()
        # print("read img",end1-start1)
        #保存图片
        ##cv2.imshow('image', img)
        # if self.count<5:#间隔5帧保存一张
        #     self.count +=1
        # else:
        #     self.count = 0
        #     self.img_count += 1
        #     print("========save===========")
        #     cv2.imwrite("/media/pi/BLACK/0920/"+str(self.img_count)+".jpg",img)
        
        # cv2.imshow("1",img)

        if ret == False:
            print('false') 
        # 执行代码
        if ret == True:
            img = cv2.resize(img, (self.size[0], self.size[1]), interpolation=cv2.INTER_AREA)
            # warped_img = cv2.warpPerspective(img, self.M, (1280, 720), cv2.INTER_LINEAR)#透视变换
            # warped_img = cv2.warpPerspective(img, self.M, (640, 360), cv2.INTER_LINEAR)
            warped_img = cv2.warpPerspective(img, self.M, (self.size[0], self.size[1]), cv2.INTER_LINEAR)
            #============================display=======================
            # cv2.imshow('warped_img ', warped_img )
            # plt.subplot(1,2,2)
            # plt.imshow(warped_img)
            # plt.scatter([399.,880.], [720.,720.])
            # plt.plot([305,305],[0,720])
            # plt.plot([405,405],[0,720])
            # plt.show()
            #============================display========================
            gray_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray_img,(5,5),0)
            ret1, binary_warped = cv2.threshold(blur, 0, 255, cv2.THRESH_OTSU)#大津阈值算法
            binary_warped = self.hlsLSelect(warped_img)#hsl变换
            #binary_warped = self.OTSU(warped_img)
            #============================display========================
            # cv2.imshow('binary_warped1 ', binary_warped )
            #===========================================================
            #===========================滑窗窗口设置====================
            nwindows = 20 #滑窗个数
            #margin = 30 #半宽值 中心点道左右边界距离
            margin = 15
            minpix = 12
            #==========================================================
            # end0 = time.time()
            # print("=====get0====",end0-start1)
            # start1 = time.time()
            lane_x, lane_y, midpoint_x = self.find_lane_pixels(binary_warped, nwindows, margin, minpix)#获取车道线像素 0.08s
            # end1 = time.time()
            # print("=====get====",end1-start1)
            
            # start2 = time.time()
            self.aP[0], self.aP[1] = self.find_aim_point(lane_x, lane_y, midpoint_x)#获取目标点 0.03s
            # end2 = time.time()
            # print("=====get2====",end2-start1)
            
            
            # img_to_save = warped_img
            # self.m = self.m+1
            # cv2.circle(
                # img_to_save, (int(self.aP[0]), int(self.aP[1])), 5, (0, 0, 128), -1)
            # cv2.imwrite(
                # ''.join(['/media/pi/BLACK/0908/', str(self.m), '.jpg']), img_to_save)
            
            
            #print(self.aP)
            #================================display==================================
            cv2.circle(binary_warped, (int(self.aP[0]),int(self.aP[1])), 5, (255,255,0),-1)
            binary_warped = cv2.putText(binary_warped,"("+str(self.aP[0])+","+str(self.aP[1])+")", (50, 600), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 255, 0), 5)
            print("---mid_point_position---: (%d, %d)"%(self.aP[0],self.aP[1]))
                  # 图像，文字内容， 坐标 ，字体，大小，颜色，字体厚度
            #=========================================================================
            
            #转换成真实距离
            self.aP[0] = (self.aP[0] - midpoint_x) * x_cmPerPixel
            self.aP[1] = (self.size[1] - self.aP[1]) * y_cmPerPixel + y_offset
            
            """
            # 0920 优化直线能力
            if ((self.lastP[0] - self.aP[0])**2 + (self.aP[1] - self.lastP[1])**2 < 200):
                pass
            """
            # 计算目标点的真实坐标
            if (self.lastP[0] > 0.001 and self.lastP[1] > 0.001):
                if (((self.aP[0] - self.lastP[0]) ** 2 + (
                        self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 1):  # To avoid the mislead by walkers
                    self.aP[0] = self.lastP[0]
                    self.Timer += 1
                else:
                    self.Timer = 0

            self.lastP = self.aP[:]
            steerAngle = atan(2 * I * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + D) * (self.aP[1] + D)))#计算阿克曼角
            
            self.cam_cmd.angular.z = k * steerAngle + k0*(steerAngle - self.pre_angle)
            self.pre_angle = steerAngle
            print("steerAngle=", steerAngle)
            print("steerAngle(angular z)=", self.cam_cmd.angular.z)
            self.cmdPub.publish(self.cam_cmd)
            # self.imagePub.publish(self.cvb.cv2_to_imgmsg(binary_warped))  # binary_warped 
            self.pdFlg.publish(self.pedestrians_flag)
            # self.sound_pub.publish(self.pedestrians_flag)
            #=================================display=================================
            cv2.imshow('binary_warped', binary_warped)
            #=========================================================================
            # end3 = time.time()
            # print("======",end3-start1)
            #cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)
    
    try:
        cam = camera()
        print("is False",rospy.is_shutdown())  # FALSE
        # rospy.Subscriber("/vcu/ActualMotorSpeed",Int32,Callback_Speed)
        while not rospy.is_shutdown():
            start = time.time()
            cam.spin()
            print('betweeen == cam.spin ==')
            rate.sleep()
            end = time.time()
            print(end - start)
            
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass


