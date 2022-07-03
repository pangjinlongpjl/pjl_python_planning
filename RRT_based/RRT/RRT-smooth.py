#! /usr/bin/python3
#! -*- coding:utf-8 -*-

"""

Rapid random tree
author: pjl

"""

from calendar import THURSDAY
import matplotlib.pyplot as plt
import random
import math
import numpy as np
from numpy.lib.function_base import append

# 全局变量
PATH_RESOLUTION = 0.05  # 路径采样点距离
THRESHOLD = 10  # 离终点距离阈值


# 点
class Point:
    def __init__(self, x, y):
        self.x_ = x  # x轴坐标
        self.y_ = y  # y轴坐标
    
    # 重载减法操作
    def __sub__(self, point):
        return math.sqrt((self.x_ - point.x_) ** 2 + (self.y_ - point.y_) ** 2)


# 工具类
class Tools:
    def __init__(self):
        return
    
    # 将点集的x坐标和y坐标分离
    @classmethod
    def departPoints(self, points):
        x = []
        y = []
        for point in points:
            x.append(point.x_)
            y.append(point.y_)
        return x, y
    
    # 将坐标转化为索引
    @classmethod
    def posToIndex(self, pos, min_pos, resolution):
        return round((pos - min_pos) / resolution)

    # 将索引转化为坐标
    @classmethod
    def indexToPos(self, index, min_pos, resolution):
        return float(index * resolution + min_pos)

    # 计算启发
    @classmethod
    def calc_heuristic(self, ngoal, node, weight = 1.0):
        return weight * math.sqrt((ngoal.nx_ - node.nx_) ** 2 + (ngoal.ny_ - node.ny_) ** 2)



# 节点
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
        
        self.parent_ = None  # 上一个节点

# RRT规划器
class RRTPlanner:
    def __init__(self, start, goal, obstacle_list, rand_area, rand_rate, expand_dis, max_iter):
        self.start_ = start
        self.goal_ = goal
        self.obstacle_list_ = obstacle_list
        self.rand_area_ = rand_area
        self.rand_rate_ = rand_rate
        self.expand_dis_ = expand_dis
        self.max_iter_ = max_iter
        
    
    # 开始进行规划
    def planning(self):

        # 初始化起始节点
        self.start_direction =  math.pi/2
        self.start_node_ = Node(self.start_[0], self.start_[1])
        # 初始化终止节点
        self.goal_node_ = Node(self.goal_[0],self.goal_[1])
        # 初始化生成树
        #self.tree_ = [self.start_node_]
        self.start_point_1 = Node(self.start_node_.x_+ self.expand_dis_*math.cos(self.start_direction),
                                    self.start_node_.y_+ self.expand_dis_*math.sin(self.start_direction))

        self.start_point_1 = self.expandNewNode(self.start_node_, self.start_point_1)
        
        self.tree_ = [self.start_point_1]
        #self.tree_.append(self.start_point_1)

        # 开始进行循环搜索
        i = 0
        while i < self.max_iter_:
            # 首先找到采样点
            random_node = self.getRandomNode()
            # 判断生成树中离采样点最近的点
            nearest_node_index = self.getNearestNodeIndexFromTree(random_node)
            nearest_node = self.tree_[nearest_node_index]
            # 从最近节点向采样点方法进行延伸,得到新的节点
            new_node = self.expandNewNode(nearest_node, random_node)
            
            # 判断新生成的节点是否是前进节点
            theta = self.getVectorAngle(nearest_node.parent_, nearest_node, new_node)

            # 进行可视化
            self.drawGraph(random_node)

            if theta <= math.pi/2:

                # 判断从最近节点到新节点是否发生障碍物碰撞
                if not self.isCollision(new_node):
                    # 没有发生碰撞

                    # 生成光滑圆弧
                    new_node.circle_x_, new_node.circle_y_ = self.getSmoothCircle(nearest_node.parent_, nearest_node, new_node, theta)
                    # 生长光滑螺线弧
                    new_node.clothoid_x_, new_node.clothoid_y_ =self.getSmoothClothoid(nearest_node.parent_, nearest_node, new_node, theta)
                    
                    # 将节点加入生成树
                    self.tree_.append(new_node)
                    # 判断新节点是否离终点距离小于阈值
                    if self.calcNodesDistance(new_node, self.goal_node_) < THRESHOLD:
                        # 到达终点
                        # 计算最终路径
                        return self.getFinalPath(new_node)
            i += 1

    # 计算两个向量的夹角
    def getVectorAngle(self,node_1, node_2, node_3):
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

    def getSmoothCircle(self, node_1, node_2, node_3, turn_theta):
        

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

        for sample in np.arange(0.0, L + PATH_RESOLUTION, PATH_RESOLUTION):
            
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
    def getSmoothClothoid(self, node_1, node_2, node_3, turn_theta):



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
        for sample_clo in np.arange(0.0, L_s + PATH_RESOLUTION, PATH_RESOLUTION):

            
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
        for sample_cir in np.arange(0.0, L_R + PATH_RESOLUTION, PATH_RESOLUTION):

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
        for sample_clo_1 in np.arange(0.0, L_s + PATH_RESOLUTION, PATH_RESOLUTION):

            
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

            clothoid_x.append(c_x)
            clothoid_y.append(c_y)



        return clothoid_x, clothoid_y


    # 找到采样点
    def getRandomNode(self):
        rand = random.random()
        if rand > self.rand_rate_:
            # 取终点
            return Node(self.goal_[0],self.goal_[1])
        else:
            # 取随机点
            return Node(random.uniform(self.rand_area_[0], self.rand_area_[1]), random.uniform(self.rand_area_[0], self.rand_area_[1]))
    
    # 获得新的节点
    def expandNewNode(self, init_node, end_node):
        # 计算初始节点到结束节点的距离和朝向
        distance = self.calcNodesDistance(init_node, end_node)
        theta = np.arctan2(end_node.y_ - init_node.y_, end_node.x_ - init_node.x_)
        # 计算从初始节点到结束节点的路径
        #distance = min(distance, self.expand_dis_)
        distance = self.expand_dis_
        x, y = init_node.x_, init_node.y_
        path_x, path_y = [], []
        print ("init_node.x_ ", init_node.x_, "init_node.y_ ", init_node.y_)
        for sample in np.arange(0.0, distance + PATH_RESOLUTION, PATH_RESOLUTION):
            x = sample * np.cos(theta) + init_node.x_
            y = sample * np.sin(theta) + init_node.y_
            print ("x ", x, "y ", y)
            path_x.append(x)
            path_y.append(y)
        # 构造新的节点
        new_node = Node(x, y)
        #new_node.path_x_= path_x[:-1]
        #new_node.path_y_ = path_y[:-1]
        new_node.path_x_= path_x
        new_node.path_y_ = path_y
        new_node.parent_ = init_node
        return new_node

    # 判断节点是否发生碰撞
    def isCollision(self, node):
        # 计算节点路径上每一个点到障碍物距离是否小于阈值
        for ix, iy in zip(node.path_x_, node.path_y_):
            for obs in self.obstacle_list_:
                distance = np.sqrt((ix - obs[0]) ** 2 + (iy - obs[1]) ** 2)
                if distance <= obs[2]+5:
                    return True
        return False
    
    # 得到最终路径
    def getFinalPath(self, final_node):
        path_x, path_y = [], []
        node = final_node
        while node.parent_ is not None:
            path_x = node.path_x_ + path_x
            path_y = node.path_y_ + path_y
            node = node.parent_
        return path_x, path_y

    # 计算生成树中离给出节点最近的节点下标
    def getNearestNodeIndexFromTree(self, node):
        distances = [self.calcNodesDistance(node, tree_node) for tree_node in self.tree_]
        min_index = distances.index(min(distances))
        return min_index

    # 计算两点之间的距离
    def calcNodesDistance(self, node_1, node_2):
        return np.sqrt((node_1.x_ - node_2.x_) ** 2 + (node_1.y_ - node_2.y_) ** 2)

    # 可视化
    def drawGraph(self, rnd = None):
        # 清空之前可视化
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x_, rnd.y_, "^k")
        for node in self.tree_:
            if node.parent_ is not None:
                plt.plot(node.path_x_, node.path_y_, "-g")
                if node.parent_.parent_:
                    plt.plot(node.circle_x_, node.circle_y_, "-b")
                    plt.plot(node.clothoid_x_, node.clothoid_y_, "-r")

        for (ox, oy, size) in self.obstacle_list_:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start_node_.x_, self.start_node_.y_, "xr")
        plt.plot(self.goal_node_.x_, self.goal_node_.y_, "xr")
        plt.axis([self.rand_area_[0], self.rand_area_[1], self.rand_area_[0], self.rand_area_[1]])
        plt.grid(True)
        plt.gca().set_aspect(1)
        plt.pause(1)

# 主函数
def main():

    # 设定起点和终点
    #start = Point(0.0, -45.0)
    #goal = Point(0.0, 45.0)

    #初始化起点,终点信息
    start = [0.0, -95.0]
    goal = [0.0, 95.0]

    # # 初始化边界障碍物信息
    # # 构建障碍物
    # obstacles = []
    # # 边界障碍物
    # for ox in range(-50, 51):
    #     for oy in range(-50, 51):
    #         if ox == -50:
    #             obstacles.append(Point(ox, oy))
    #         elif ox == 50:
    #             obstacles.append(Point(ox, oy))
    #         elif oy == -50:
    #             obstacles.append(Point(ox, oy))
    #         elif oy == 50:
    #             obstacles.append(Point(ox, oy))
    #         # elif ox == 20 and oy < 70 and oy > 5:
    #         #     obstacles.append(Point(ox, oy))
    #         # elif ox == 30 and oy < 70 and oy > 5:
    #         #     obstacles.append(Point(ox, oy))
    #         # elif oy == 70 and ox < 30 and ox > 20:
    #         #     obstacles.append(Point(ox, oy))
    # # 对当前信息进行可视化
    # obstalces_x, obstacles_y = Tools.departPoints(obstacles)
    # plt.plot(obstalces_x, obstacles_y, ".k")
    # plt.plot(start[0], start[1], "og")
    # plt.plot(goal[0], goal[1], "xb")
    # #plt.grid(True)
    # plt.axis("equal")

    obstacle_list = [
        # (0, 20, 1),
        # (0, -20, 1),

        # (5, 20, 1),
        # (5, -20, 1),

        # (-5, 20, 1),
        # (-5, -20, 1),
        # (3, 8, 2),
        # (3, 10, 2),
        # (7, 5, 2),
        # (9, 5, 2)
    ]  # [x,y,size]
    # 初始化采样
    rand_area=[-100.0, 100.0]
    # 初始化步长
    expand_dis = 20.0
    # 初始化最大迭代次数
    max_iter = 100
    # 初始化随机点采样概率
    rand_rate = 0.9

    # 开始规划
    rrt_planner = RRTPlanner(start, goal, obstacle_list, rand_area, rand_rate, expand_dis, max_iter)
    path_x, path_y = rrt_planner.planning()

    # 进行可视化
    rrt_planner.drawGraph()
    # plt.plot(path_x, path_y, "-r")
    plt.grid(True)
    plt.gca().set_aspect(1)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()
    #toward_angle = np.arctan2(np.sqrt(3), 1)
    #print("方位角",toward_angle)