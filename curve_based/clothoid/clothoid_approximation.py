#! /usr/bin/python3
#! -*- coding:utf-8 -*-

"""
clothoid_approximation
author: pjl

"""

from distutils.ccompiler import gen_lib_options
import matplotlib.pyplot as plt
import math
import numpy as np

# 全局变量
PATH_RESOLUTION = 0.05 # 路径采样点距离

class Node:
    def __init__(self, x, y):
        self.x_ = x  # 节点的x坐标
        self.y_ = y  # 节点的y坐标
        self.path_x_ = []  # 从上一点到本节点的路径x坐标
        self.path_y_ = []  # 从上一点到本节点的路径y坐标
        self.circle_x_ = [] # 平滑的圆弧x坐标
        self.circle_y_ = [] # 平滑的圆弧y坐标
        self.clothoid_x_ = [] # 平滑的螺线x坐标
        self.clothoid_y_ = [] # 平滑的螺线y坐标

# 计算两个向量的夹角
def getVectorAngle(node_1, node_2, node_3):
        ## 得到两个向量
    vector_1 = np.array([node_2.x_ - node_1.x_, node_2.y_ - node_1.y_])
    vector_2 = np.array([node_3.x_ - node_2.x_, node_3.y_ - node_2.y_])
    
    # 分别计算两个向量的模：
    l_vector_1=np.sqrt(vector_1.dot(vector_1))
    l_vector_2=np.sqrt(vector_2.dot(vector_2))
    #print('向量的模=',l_vector_1,l_vector_2)

    # 计算两个向量的点积
    dian=vector_1.dot(vector_2)
    #print('向量的点积=',dian)

    # 计算夹角的cos值：
    cos_=dian/(l_vector_1*l_vector_2)
    #print('夹角的cos值=',cos_)

    # 求得夹角（弧度制）：
    angle_hu=np.arccos(cos_)
    #print('夹角（弧度制）=',angle_hu)


    # # 转换为角度值：
    # angle_d=angle_hu*180/np.pi
    # print('夹角=%f°'%angle_d) 

    return angle_hu

# 得到平滑后的圆曲线
def getSmoothCircle(node_1, node_2, node_3, turn_theta):
    

    vector_1 = np.array([node_2.x_ - node_1.x_, node_2.y_ - node_1.y_])
    vector_2 = np.array([node_3.x_ - node_2.x_, node_3.y_ - node_2.y_])

    R = 5
    #首先计算切线的长度
    T = R * math.tan(turn_theta*0.5)
    #然后计算圆曲线的弧长
    L = R * turn_theta

    #圆弧插值

    #坐标变换的角度
    toward_angle_1 = np.arctan2(node_2.y_- node_1.y_, node_2.x_- node_1.x_)

    toward_angle_2 = np.arctan2(node_3.y_- node_2.y_, node_3.x_- node_2.x_)

    if toward_angle_1<0:
        toward_angle_1 += 2*math.pi
    
    if toward_angle_2<0:
        toward_angle_2 += 2*math.pi


    #切点坐标
    T_x_1 = node_2.x_ - T*np.cos(toward_angle_1)
    T_y_1 = node_2.y_ - T*np.sin(toward_angle_1)

    T_x_2 = node_2.x_ + T*np.cos(toward_angle_2)
    T_y_2 = node_2.y_ + T*np.sin(toward_angle_2)

    circle_x = []
    circle_y = []

        #两个向量的叉乘
    vector_cross = np.cross(vector_1, vector_2)

    #print(vector_cross)

    # 右手系
    if vector_cross > 0:
        circle_center_x  = T_x_1 + R*np.cos(toward_angle_1+ math.pi*0.5)
        circle_center_y  = T_y_1 + R*np.sin(toward_angle_1+ math.pi*0.5)
    # 左手系
    else:
        circle_center_x  = T_x_1 + R*np.cos(toward_angle_1- math.pi*0.5)
        circle_center_y  = T_y_1 + R*np.sin(toward_angle_1- math.pi*0.5)

    for sample in np.arange(0.0, L - PATH_RESOLUTION, PATH_RESOLUTION):
        
        fai = sample/L*turn_theta

        #已知圆上两点及弧长，圆弧插值
        x = np.sin(turn_theta-fai)/np.sin(turn_theta)*(T_x_1 - circle_center_x) + np.sin(fai)/np.sin(turn_theta)* (T_x_2 - circle_center_x)
        y = np.sin(turn_theta-fai)/np.sin(turn_theta)*(T_y_1 - circle_center_y) + np.sin(fai)/np.sin(turn_theta)* (T_y_2 - circle_center_y)
            
        x = x+ circle_center_x
        y = y+ circle_center_y


        circle_x.append(x)
        circle_y.append(y)


    sample = L
    fai = sample/L*turn_theta

    #已知圆上两点及弧长，圆弧插值
    x = np.sin(turn_theta-fai)/np.sin(turn_theta)*(T_x_1 - circle_center_x) + np.sin(fai)/np.sin(turn_theta)* (T_x_2 - circle_center_x)
    y = np.sin(turn_theta-fai)/np.sin(turn_theta)*(T_y_1 - circle_center_y) + np.sin(fai)/np.sin(turn_theta)* (T_y_2 - circle_center_y)
        
    x = x+ circle_center_x
    y = y+ circle_center_y

    circle_x.append(x)
    circle_y.append(y)

    #另一种圆弧插值的坐标变换方法

    # toward_angle = np.arctan2(node_2.y_- node_1.y_, node_2.x_- node_1.x_)

    # if toward_angle<0:
    #     toward_angle += 2*math.pi

    # #切点坐标
    # T_x = node_2.x_ - T*np.cos(toward_angle)
    # T_y = node_2.y_ - T*np.sin(toward_angle)

    # #两个向量的叉乘
    # vector_cross = np.cross(vector_1, vector_2)

    # #print(vector_cross)

    # circle_x = []
    # circle_y = []

    # if vector_cross > 0:

    #     for sample in np.arange(0.0, L + PATH_RESOLUTION, PATH_RESOLUTION):
        
    #         fai = sample/L*turn_theta
    #         x = R*np.sin(fai)
    #         y = R*(1-np.cos(fai))
    #         c_x = T_x + x*np.cos(toward_angle)- (y*np.sin(toward_angle))
    #         c_y = T_y + x*np.sin(toward_angle)+ y*np.cos(toward_angle)
    #         circle_x.append(c_x)
    #         circle_y.append(c_y)
    # else:

    #     for sample in np.arange(0.0, L + PATH_RESOLUTION, PATH_RESOLUTION):
        
    #         fai = sample/L*turn_theta
    #         x = R*np.sin(fai)
    #         y = -R*(1-np.cos(fai))
    #         c_x = T_x + x*np.cos(toward_angle)- (y*np.sin(toward_angle))
    #         c_y = T_y + x*np.sin(toward_angle)+ y*np.cos(toward_angle)

    #         circle_x.append(c_x)
    #         circle_y.append(c_y)

    return circle_x, circle_y

# 得到平滑后的螺旋曲线
def getSmoothClothoid(node_1, node_2, node_3, turn_theta):

    vector_1 = np.array([node_2.x_ - node_1.x_, node_2.y_ - node_1.y_])
    vector_2 = np.array([node_3.x_ - node_2.x_, node_3.y_ - node_2.y_])

    #坐标变换的角度
    toward_angle_1 = np.arctan2(node_2.y_- node_1.y_, node_2.x_- node_1.x_)

    toward_angle_2 = np.arctan2(node_3.y_- node_2.y_, node_3.x_- node_2.x_)

    if toward_angle_1<0:
        toward_angle_1 += 2*math.pi
    
    if toward_angle_2<0:
        toward_angle_2 += 2*math.pi

    #两个向量的叉乘
    vector_cross = np.cross(vector_1, vector_2)

    clothoid_x = []
    clothoid_y = []
    
    #首先计算一些基本量

    #圆曲线半径
    R = 10
    #缓和曲线长
    L_s = 2

    m = L_s/2 - L_s*L_s*L_s/(240*R*R)
    P = L_s*L_s/(24*R)
    beta = L_s/(2*R)

    #切线长
    T_H = m + (R+P)*math.tan(turn_theta/2)
    #圆弧曲线长
    L_R = R*(turn_theta - 2*beta)
    #曲线长
    L_H = R*(turn_theta - 2*beta) + 2*L_s
    #外矢距
    E_H = (R+P)/math.cos(turn_theta/2) - R
    #切曲差
    q = 2*T_H - L_H

        #切点坐标
    T_x_1 = node_2.x_ - T_H*np.cos(toward_angle_1)
    T_y_1 = node_2.y_ - T_H*np.sin(toward_angle_1)

    T_x_2 = node_2.x_ + T_H*np.cos(toward_angle_2)
    T_y_2 = node_2.y_ + T_H*np.sin(toward_angle_2)


    #缓和曲线（螺线）独立坐标简化计算公式
    for sample_clo in np.arange(0.0, L_s - PATH_RESOLUTION, PATH_RESOLUTION):

        
        x = sample_clo - math.pow(sample_clo,5)/(40*R*R*L_s*L_s)

        # 左手系
        if vector_cross > 0:
            y = math.pow(sample_clo,3)/(6*R*L_s)

        # 右手系
        else:
            y = -(math.pow(sample_clo,3)/(6*R*L_s))

        #坐标变换到全局坐标系中
        c_x = T_x_1 + x*np.cos(toward_angle_1) - (y*np.sin(toward_angle_1))
        c_y = T_y_1 + x*np.sin(toward_angle_1) + (y*np.cos(toward_angle_1))

        clothoid_x.append(c_x)
        clothoid_y.append(c_y)


    #圆曲线段独立坐标计算公式
    for sample_cir in np.arange(0.0, L_R - PATH_RESOLUTION, PATH_RESOLUTION):

        fai = beta + sample_cir/R 
        # fai = (sample_cir - 0.5*L_s)/R

        x = m + R * math.sin(fai)

        # 左手系
        if vector_cross > 0:
            y = P + R * (1 - math.cos(fai)) 

        # 右手系
        else :
            y = -(P + R * (1 - math.cos(fai))) 

        #坐标变换到全局坐标系中
        c_x = T_x_1 + x*np.cos(toward_angle_1) - (y*np.sin(toward_angle_1))
        c_y = T_y_1 + x*np.sin(toward_angle_1) + (y*np.cos(toward_angle_1))

        #print ("c_x", c_x)
        #print ("c_y", c_y)

        clothoid_x.append(c_x)
        clothoid_y.append(c_y)

    #缓和曲线对称段坐标
    clothoid_x_2 = []
    clothoid_y_2 = []
    for sample_clo_1 in np.arange(0.0, L_s - PATH_RESOLUTION, PATH_RESOLUTION):

        
        x = sample_clo_1 - math.pow(sample_clo_1,5)/(40*R*R*L_s*L_s)

        # 左手系
        if vector_cross < 0:
            y = math.pow(sample_clo_1,3)/(6*R*L_s)

        # 右手系
        else:
            y = -math.pow(sample_clo_1,3)/(6*R*L_s)

        #坐标变换到全局坐标系中

        toward_angle_3 = np.arctan2(node_2.y_- node_3.y_, node_2.x_- node_3.x_)

        if toward_angle_3<0:
            toward_angle_3 += 2*math.pi

        c_x = T_x_2 + x*np.cos(toward_angle_3) - (y*np.sin(toward_angle_3))
        c_y = T_y_2 + x*np.sin(toward_angle_3) + (y*np.cos(toward_angle_3))

        clothoid_x_2.append(c_x)
        clothoid_y_2.append(c_y)

    clothoid_x.extend(clothoid_x_2[::-1])
    clothoid_y.extend(clothoid_y_2[::-1])



    return clothoid_x, clothoid_y

def main():

    start_node = Node(-15, 0)
    middle_node = Node(0, 0)
    end_node = Node(0, 15)

    node_x = [-15, 0, 0]
    node_y = [0, 0, 15]

    theta = getVectorAngle(start_node, middle_node, end_node)
    end_node.circle_x_, end_node.circle_y_ = getSmoothCircle(start_node, middle_node, end_node, theta)
    end_node.clothoid_x_, end_node.clothoid_y_ = getSmoothClothoid(start_node, middle_node, end_node, theta)

    plt.clf()
    plt.plot(node_x, node_y, "-g")
    plt.plot(end_node.circle_x_, end_node.circle_y_, "-b")
    plt.plot(end_node.clothoid_x_, end_node.clothoid_y_, "-r")
    plt.grid(True)
    plt.gca().set_aspect(1)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()