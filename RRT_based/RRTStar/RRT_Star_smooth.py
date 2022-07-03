#! /usr/bin/python3
#! -*- coding: utf-8 -*-

"""

Rapid random tree star
author: pjl

"""

import matplotlib.pyplot as plt
import random
import math
import numpy as np

# 全局变量
PATH_RESOLUTION = 0.05  # 路径采样点距离
THRESHOLD = 50  # 离终点距离阈值

# 节点
class Node:
    def __init__(self, x, y):
        self.x_ = x  # 节点的x坐标
        self.y_ = y  # 节点的y坐标
        self.path_x_ = []  # 从上一点到本节点的路径x坐标
        self.path_y_ = []  # 从上一点到本节点的路径y坐标
        self.circle_x_ = [] # 平滑的圆弧x坐标
        self.circle_y_ = [] # 平滑的圆弧y坐标
        self.clothoid_x_ = [] #平滑的螺线x坐标
        self.clothoid_y_ = [] #平滑的螺线y坐标
        self.parent_ = None  # 上一个节点
        self.parent_id_ = 0 # 父亲节点的id
        self.cost_ = 0.0  # 代价

# RRT规划器
class RRTStarPlanner:
    def __init__(self, start, goal, obstacle_list, rand_area, rand_rate, expand_dis, connect_circle_dist, max_iter):
        self.start_ = start
        self.goal_ = goal
        self.obstacle_list_ = obstacle_list
        self.rand_area_ = rand_area
        self.rand_rate_ = rand_rate
        self.expand_dis_ = expand_dis
        self.max_iter_ = max_iter
        self.connect_circle_dist_ = connect_circle_dist
    
    # 开始进行规划
    def planning(self):
        # 初始化起始节点
        self.start_direction = math.pi/2
        self.start_node_ = Node(self.start_[0], self.start_[1])
        # 初始化终止节点
        self.goal_node_ = Node(self.goal_[0],self.goal_[1])
        # 初始化生成树
        self.start_point_1 = Node(self.start_node_.x_ + self.expand_dis_ * math.cos(self.start_direction),
                                    self.start_node_.y_ + self.expand_dis_ * math.sin(self.start_direction))

        self.start_point_1 = self.expandNewNode(self.start_node_, self.start_point_1)

        self.tree_ = [self.start_point_1]
        # 开始进行循环搜索
        i = 0
        while i < self.max_iter_:
            print("Iter:", i, ", number of nodes:", len(self.tree_))
            # 首先找到采样点
            random_node = self.getRandomNode()
            # 判断生成树中离采样点最近的点
            nearest_node_index = self.getNearestNodeIndexFromTree(random_node)
            nearest_node = self.tree_[nearest_node_index]
            # 从最近节点向采样点方法进行延伸,得到新的节点
            new_node = self.expandNewNode(nearest_node, random_node, self.expand_dis_)
            # 找到新节点的在生成树中的邻居节点
            neighbor_node_indexes = self.findNeighborNodeIndexes(new_node)
            # 为新节点更新父节点
            new_node = self.updateParentForNewNode(new_node, neighbor_node_indexes)

            # 进行可视化
            #if i % 10 == 0:
            self.drawGraph(random_node)
            # 判断新节点是否有效
            if new_node is not None:

                # 判断新生成的节点是否是前进节点
                theta = self.getVectorAngle(new_node.parent_.parent_, new_node.parent_, new_node)

                if theta <= math.pi/2 and theta >= math.pi/6:

                    #生成光滑圆弧
                    new_node.circle_x_, new_node.circle_y_ = self.getSmoothCircle(new_node.parent_.parent_, new_node.parent_, new_node, theta)

                    #生成光滑螺线弧
                    new_node.clothoid_x_, new_node.clothoid_y_ = self.getSmoothClothoid(new_node.parent_.parent_, new_node.parent_, new_node, theta)

                    # 将新节点加入生成树
                    self.tree_.append(new_node)

                    # 新节点有效,更新邻居节点的连接
                    update_indexes = self.updateWire(new_node, neighbor_node_indexes) 

                    #为更新后的邻居节点生成光滑圆弧和光滑螺线弧
                    for update_index in update_indexes:
                        tmp_node = self.tree_[update_index]
                        
                        #先更新邻居节点的子节点信息，然后更新该节点信息
                        #为更新后的邻居节点的子节点生成光滑圆弧和光滑螺线弧
                        #遍历全部的生成树节点
                        for j, tree_node in enumerate(self.tree_):
                            if tree_node.parent_ == tmp_node:
                                print("get tree node ")
                                tree_node = self.tree_[j]
                                # 找到了子节点，为子节点更新光滑圆弧和光滑螺线弧
                                tree_theta = self.getVectorAngle(tree_node.parent_.parent_, tree_node.parent_, tree_node)
                                tree_node.circle_x_, tree_node.circle_y_ = self.getSmoothCircle(tree_node.parent_.parent_, tree_node.parent_, tree_node, tree_theta)
                                tree_node.clothoid_x_, tree_node.clothoid_y_ = self.getSmoothClothoid(tree_node.parent_.parent_, tree_node.parent_, tree_node, tree_theta)
                                self.tree_[j] = tree_node

                        tmp_theta = self.getVectorAngle(tmp_node.parent_.parent_, tmp_node.parent_, tmp_node)
                        tmp_node.circle_x_, tmp_node.circle_y_ = self.getSmoothCircle(tmp_node.parent_.parent_, tmp_node.parent_, tmp_node, tmp_theta)
                        tmp_node.clothoid_x_, tmp_node.clothoid_y_ = self.getSmoothClothoid(tmp_node.parent_.parent_, tmp_node.parent_, tmp_node, tmp_theta)
                        self.tree_[update_index] = tmp_node

                        # #为更新后的邻居节点的子节点生成光滑圆弧和光滑螺线弧
                        # #遍历全部的生成树节点
                        # for j, tree_node in enumerate(self.tree_):
                        #     if tree_node.parent_ == tmp_node:
                        #         print("get tree node ")
                        #         tree_node = self.tree_[j]
                        #         # 找到了子节点，为子节点更新光滑圆弧和光滑螺线弧
                        #         tree_theta = self.getVectorAngle(tree_node.parent_.parent_, tree_node.parent_, tree_node)
                        #         tree_node.circle_x_, tree_node.circle_y_ = self.getSmoothCircle(tree_node.parent_.parent_, tree_node.parent_, tree_node, tree_theta)
                        #         tree_node.clothoid_x_, tree_node.clothoid_y_ = self.getSmoothClothoid(tree_node.parent_.parent_, tree_node.parent_, tree_node, tree_theta)
                        #         self.tree_[j] = tree_node

            i += 1
        # 遍历完成,开始得到最终路径
        last_index = self.findFinalNode()
        if last_index is None:
            print('no path find')
            exit(0)
        else:
            return self.getFinalPath(self.tree_[last_index])

    #计算两个向量的夹角
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
    def expandNewNode(self, init_node, end_node, max_distance=float('inf')):
        # 计算初始节点到结束节点的距离和朝向
        distance = self.calcNodesDistance(init_node, end_node)
        theta = np.arctan2(end_node.y_ - init_node.y_, end_node.x_ - init_node.x_)
        # 计算从初始节点到结束节点的路径
        #print ("max_distance ", max_distance)
        distance = min(distance, max_distance)
        #distance = min(distance, self.expand_dis_)
        #distance = self.expand_dis_
        x, y = init_node.x_, init_node.y_
        path_x, path_y = [], []
        for sample in np.arange(0.0, distance + PATH_RESOLUTION, PATH_RESOLUTION):
            x = sample * np.cos(theta) + init_node.x_
            y = sample * np.sin(theta) + init_node.y_
            path_x.append(x)
            path_y.append(y)
        # 构造新的节点
        new_node = Node(x, y)
        #new_node.path_x_= path_x[:-1]
        #new_node.path_y_ = path_y[:-1]
        new_node.path_x_= path_x
        new_node.path_y_ = path_y
        new_node.parent_ = init_node
        new_node.cost_ = self.calcCost(init_node, new_node)
        return new_node

    # 判断节点是否发生碰撞
    def isCollision(self, node):
        # 计算节点路径上每一个点到障碍物距离是否小于阈值
        for ix, iy in zip(node.path_x_, node.path_y_):
            for obs in self.obstacle_list_:
                distance = np.sqrt((ix - obs[0]) ** 2 + (iy - obs[1]) ** 2)
                if distance <= obs[2]:
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
    
    # 计算节点的代价
    def calcCost(self, init_node, end_node):
        return init_node.cost_ + self.calcNodesDistance(init_node, end_node)

    # 寻找邻居节点
    def findNeighborNodeIndexes(self, node):
        # 得到搜索半径
        radius = self.connect_circle_dist_ * np.sqrt(np.log(len(self.tree_) + 1) / (len(self.tree_) + 1))
        indexes = []
        for i, tree_node in enumerate(self.tree_):
            distance = self.calcNodesDistance(node, tree_node)
            if distance <= radius:
                indexes.append(i)
        return indexes
    
    # 为新节点更新父节点
    def updateParentForNewNode(self, node, neighbor_node_indexes):
        # 遍历邻居节点
        valid_temp_node = []
        for neighbor_node_index in neighbor_node_indexes:
            neighbor_node = self.tree_[neighbor_node_index]
            # 构建临时节
            temp_node = self.expandNewNode(neighbor_node, node, self.expand_dis_)
            # 判断临时节点是否发生碰撞
            if not self.isCollision(temp_node):
                # 如果没有发生碰撞,加入列表中
                valid_temp_node.append(temp_node)
        if len(valid_temp_node) == 0:
            # 没有有效节点
            return None
        else:
            # 返回代价最小的
            min_cost_node = min(valid_temp_node, key = lambda temp_node: temp_node.cost_)
            return min_cost_node

    # 更新新的连接关系
    def updateWire(self, node, neighbor_node_indexes):
        
        update_indexes = []
        # 遍历邻居节点
        for neighbor_node_index in neighbor_node_indexes:
            neighbor_node = self.tree_[neighbor_node_index]
            # 构建临时节点
            temp_node = self.expandNewNode(node, neighbor_node)
            # 计算临时节点的夹角
            theta = self.getVectorAngle(temp_node.parent_.parent_, temp_node.parent_, temp_node)
            # 判断是否发生碰撞
            if self.isCollision(temp_node):
                # 如果发生碰撞,跳过
                continue
            # 如果不发生碰撞,判断代价
            if temp_node.cost_ < neighbor_node.cost_:
                # 如果新的节点代价更低,更新之前的邻居节点为新的
                # 判断新生成的节点是否是前进节点
                if theta <= math.pi/2 and theta >= math.pi/6:
                    self.tree_[neighbor_node_index] = temp_node
                    # 更新该邻居的全部子节点
                    self.propegateCost(neighbor_node, neighbor_node_index)
                    update_indexes.append(neighbor_node_index)

        return update_indexes
    
    # 向子节点传播损失
    def propegateCost(self, node, neighbor_node_index):
        # 遍历全部的生成树节点
        for i, tree_node in enumerate(self.tree_):
            if tree_node.parent_ == node:
                #将原来的父节点更换为改动后的父节点
                tree_node.parent_ = self.tree_[neighbor_node_index]
                # 找到了子节点,进行损失更新
                self.tree_[i].cost_ = self.calcCost(self.tree_[neighbor_node_index], self.tree_[i])
                self.propegateCost(self.tree_[i], i)

    # 寻找离终点最近的节点
    def findFinalNode(self):
        final_indexes = []
        for i, tree_node in enumerate(self.tree_):
            distance = self.calcNodesDistance(tree_node, self.goal_node_)
            if distance < THRESHOLD:
                final_indexes.append(i)
        # 判断是否找到终点
        if len(final_indexes) == 0:
            # 没有找到终点
            return None
        final_index = min(final_indexes, key = lambda index: self.tree_[index].cost_)
        return final_index

    # 可视化
    def drawGraph(self, rnd = None):
        # 清空之前可视化
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x_, rnd.y_, "^k")
        for node in self.tree_:
            if node.parent_:
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
        plt.pause(0.5)

# 主函数
def main():
    # 初始化起点,终点信息
    start = [0.0, -95.0]
    goal = [0.0, 95.0]
    # 初始化障碍物信息
    obstacle_list = [
        # (5, 5, 1),
        # (3, 6, 2),
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
    max_iter = 300
    # 初始化随机点采样概率
    rand_rate = 0.9
    # 初始邻居半径
    connect_circle_dist = 75.0

    # 开始规划
    rrt_star_planner = RRTStarPlanner(start, goal, obstacle_list, rand_area, rand_rate, expand_dis, connect_circle_dist, max_iter)
    rrt_star_planner.planning()
    # path_x, path_y = rrt_star_planner.planning()

    # 进行可视化
    rrt_star_planner.drawGraph()
    #plt.plot(path_x, path_y, 'r')
    plt.grid(True)
    plt.gca().set_aspect(1)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()