#!/usr/bin/python
# -*- coding: utf-8 -*-
from lane_utils import laneDetect
import numpy as np
import cv2
from bluetooth_bridge.msg import LaneMsg, Sensors
import rospy
from cap_init import CapInit
import threading
from maindetection import laneDet


slope_flag = 0 # 0表示平地，1表示下坡

def main():
    rospy.init_node('lane_node', anonymous=True)
    cap, t = CapInit()


    print("[Lane Node]: Init")
    # 车道线检测对象

    #--- subscribe topic
    def sensor_callback(msg):
        """ 上下坡后重置 """
        global slope_flag
        theta = msg.thetax
        if theta < -1700 and slope_flag==0: # 检测到开始下坡
            slope_flag = 1
        elif abs(theta) < 200 and slope_flag==1: # 检测到下坡结束
            slope_flag = 0
            laneDet.refresh()

    rospy.Subscriber('/vcu', Sensors, sensor_callback)

    #--- publish topic
    lane_pub  = rospy.Publisher("/lane_detect", LaneMsg, queue_size=10)

    ros_spin = threading.Thread(target = rospy.spin)
    ros_spin.setDaemon(True)
    ros_spin.start()

    while True:

        # try:
        _, bgr_img = cap.read()
        if bgr_img is None:
            break

        bgr_img = cv2.resize(bgr_img, None, fx=0.5, fy=0.5)

        result, bgr_img, offset, _, _ = laneDet.feedCap(bgr_img)
        result = cv2.merge([result, result, result])
        result = np.vstack([bgr_img, result])
        cv2.imshow("sjb", result)
        cv2.waitKey(t)
        if cv2.getWindowProperty("sjb", cv2.WND_PROP_AUTOSIZE) < 1:
            break

    #cap.release()
    while not rospy.is_shutdown():
        # try:
        _, bgr_img = cap.read()
        if bgr_img is None:
            break

        bgr_img = cv2.resize(bgr_img, None, fx=0.5, fy=0.5)

        gear = int (offset * 100) + 50
        result, bgr_img, offset, _, _ = laneDet.feedCap(bgr_img)
        gear = int (offset * 100) + 50
        #result = cv2.merge([result, result, result])
        #result = np.vstack([bgr_img, result])
        #cv2.imshow("sjb", result)
        cv2.waitKey(t)
        if cv2.getWindowProperty("sjb", cv2.WND_PROP_AUTOSIZE) < 1:
            break

        #if _:
            #bias, gear = laneDet.spin(bgr_img)
        lane_pub.publish(LaneMsg(bias=0, gear=gear))
        #else:
            #print("error")


if __name__ == '__main__':
    main()

"""#!/usr/bin/python
# -*- coding: utf-8 -*-
from lane_utils import laneDetect
import numpy as np
import cv2
from bluetooth_bridge.msg import LaneMsg, Sensors
import rospy
from cap_init import CapInit
import threading
from lane_utils2 import laneDet


slope_flag = 0 # 0表示平地，1表示下坡

def main():
    rospy.init_node('lane_node', anonymous=True)
    cap, t = CapInit()

    print("[Lane Node]: Init")
    # 车道线检测对象

    #--- subscribe topic
    def sensor_callback(msg):
        """ 上下坡后重置 """
        global slope_flag
        theta = msg.thetax
        if theta < -1700 and slope_flag==0: # 检测到开始下坡
            slope_flag = 1
        elif abs(theta) < 200 and slope_flag==1: # 检测到下坡结束
            slope_flag = 0
            laneDet.refresh()

    rospy.Subscriber('/vcu', Sensors, sensor_callback)

    #--- publish topic
    lane_pub  = rospy.Publisher("/lane_detect", LaneMsg, queue_size=10)

    ros_spin = threading.Thread(target = rospy.spin)
    ros_spin.setDaemon(True)
    ros_spin.start()

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if ret:
            bias, gear = laneDet.spin(img)
            lane_pub.publish(LaneMsg(bias=bias, gear=gear))

if __name__ == '__main__':
    main()"""
