#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
class Map():
    def __init__(self,mapParam):
        self.sensorAngleMax = mapParam['sensorAngleMax']     ##传感器测量角度的最大值
        self.sensorAngleMin = mapParam['sensorAngleMin']     ##传感器测量角度的最小值
        self.sensorAngleReso = mapParam['sensorAngleReso']   ##传感器测量角度的最小分辨率
        self.followAngleS = mapParam['followAngleS']         ##需要进行数据处理的传感器测量最小角度
        self.followAngleE = mapParam['followAngleE']         ##需要进行数据处理的传感器测量最大角度
        self.adjacentRange = mapParam['adjacentRange']       ##需要进行数据处理的传感器探测距离的最大值
        self.mapGridSize = mapParam['mapGridSize']           ##栅格地图的栅格尺寸
        self.gridNumThres = mapParam['gridNumThres']         ##栅格地图中每个栅格内存取传感器坐标点个数的最小值，大于最小值，认为栅格被占据，否则栅格空闲
        self.globalPointOrd = []
        self.mapType = mapParam['mapType']                   ##地图输出类型，1为栅格地图，0为传感器坐标点地图(被占据栅格内的坐标点构成)

    def getScan(self):
        self.sensorPoint = []
        ##获取传感器话题数据
        topic_name=rospy.get_param('~scan_topic')
        msg = rospy.wait_for_message(topic_name, LaserScan)
        for i in range(len(msg.ranges)):
            tmp = self.sensorAngleMin + i * self.sensorAngleReson
            ##检测这个角度是否在需要数据处理的角度范围内，以及这个角度是否在允许距离范围内
            if tmp >= self.followAngleS and tmp <= self.followAngleE and msg.ranges[i] <= self.adjacentRange:
                if msg.ranges[i] == float('inf'):
                    continue
                else:
                    ##转化成坐标点
                    x = msg.ranges[i] * math.cos(tmp)
                    y = msg.ranges[i] * math.sin(tmp)
                    self.sensorPoint.append([round(x, 3), round(y, 3)])

    def getodom(self):
        ##读取里程计信息
        topic_name=rospy.get_param('~odom_topic')
        msgodom = rospy.wait_for_message(topic_name, Odometry)
        x = msgodom.pose.pose.orientation.x
        y = msgodom.pose.pose.orientation.y
        z = msgodom.pose.pose.orientation.z
        w = msgodom.pose.pose.orientation.w
        ##获取机器人位置以及角度(四元数转化)
        self.vehicleX = msgodom.pose.pose.position.x
        self.vehicleY = msgodom.pose.pose.position.y
        self.vehicleYaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def localToGlobal(self):
        ##将每次读取的传感器坐标点，转化成全局坐标系下的。
        for i in range(len(self.sensorPoint)):
            x = self.sensorPoint[i][0] * math.cos(self.vehicleYaw) - self.sensorPoint[i][1] * math.sin(self.vehicleYaw) + self.vehicleX
            y = self.sensorPoint[i][0] * math.sin(self.vehicleYaw) + self.sensorPoint[i][1] * math.cos(self.vehicleYaw) + self.vehicleY
            self.globalPointOrd.append([round(x, 3), round(y, 3)])

    def generateGridMap(self):
        ##生成栅格地图
        self.gridmap = []
        ##获取栅格地图的x,y的最大最小值，以及栅格地图的每行每列栅格数量多少
        self.maxX = max(np.array(self.globalPointOrd)[:, 0])
        self.minX = min(np.array(self.globalPointOrd)[:, 0])
        self.maxY = max(np.array(self.globalPointOrd)[:, 1])
        self.minY = min(np.array(self.globalPointOrd)[:, 1])
        self.gridNumX = int(math.ceil((self.maxX - self.minX) / self.mapGridSize) + 1)
        self.gridNumY = int(math.ceil((self.maxY - self.minY) / self.mapGridSize) + 1)
        for indy in range(self.gridNumY):
            y = self.minY + self.mapGridSize * indy
            for indx in range(self.gridNumX):
                x = self.minX + indx * self.mapGridSize
                self.gridmap.append([x, y])
        self.correspondence = [[] for i in range(len(self.gridmap))]
        ##将传感器坐标点放到对应的栅格内。
        for i in range(len(self.globalPointOrd)):
            indexX = int(((self.globalPointOrd[i][1] - self.minY) - self.mapGridSize / 2) / self.mapGridSize) + 1
            indexY = int(((self.globalPointOrd[i][0] - self.minX) - self.mapGridSize / 2) / self.mapGridSize) + 1
            index  = indexX * self.gridNumX + indexY
            self.correspondence[index].append(self.globalPointOrd[i])

    def generateGlobalMap(self):
        ##生成输出地图
        globalmap = []
        ##如果mapType == 1：输出为栅格地图;如果mapType == 0：为传感器坐标点地图(被占据栅格内的坐标点构成)
        if self.mapType:
            for i in range(len(self.correspondence)):
                if len(self.correspondence[i]) > self.gridNumThres:
                    globalmap.append(self.gridmap[i])
        else:
            for i in range(len(self.correspondence)):
                if len(self.correspondence[i]) > self.gridNumThres:
                    globalmap = globalmap + self.correspondence[i]
        return globalmap

def main():
    ##定义节点
    rospy.init_node('globalmap')
    sensorAngleMax = rospy.get_param('sensorAngleMax')
    sensorAngleMin = rospy.get_param('sensorAngleMin')
    sensorAngleReso = rospy.get_param('sensorAngleReso')
    followAngleS = rospy.get_param('followAngleS')
    followAngleE = rospy.get_param('followAngleE')
    adjacentRange = rospy.get_param('adjacentRange')
    mapGridSize = rospy.get_param('mapGridSize')
    gridNumThres = rospy.get_param('gridNumThres')
    mapType = rospy.get_param('mapType') 
    mapParam = {'sensorAngleMax':sensorAngleMax,'sensorAngleMin': sensorAngleMin,'sensorAngleReso':sensorAngleReso,'followAngleS':followAngleS,'followAngleE':followAngleE,'adjacentRange':adjacentRange,'mapGridSize':mapGridSize,'gridNumThres':gridNumThres,'mapType':mapType}
    ##初始化
    mapGen = Map(mapParam)
    ##执行接受odom，scan信息程序，并将每次接受的scan的信息转化成全局坐标系下的坐标点。
    try:
        while True:
            mapGen.getScan()
            mapGen.getodom()
            mapGen.localToGlobal()
    ##接受完成之后，生成全局地图，输出
    except KeyboardInterrupt:
        mapGen.generateGridMap()
        globalmap = mapGen.generateGlobalMap()
    plt.scatter(np.array(globalmap)[:, 0], np.array(globalmap)[:,1], color = 'r')
    plt.show()
if __name__ == '__main__':
        main()

