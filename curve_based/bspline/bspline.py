'''
Descripttion: 
version: 
Author: pjl
Date: 2022-05-29 19:57:15
LastEditors: pjl
LastEditTime: 2022-05-29 19:58:57
'''
#! /usr/bin/python3
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt
import numpy as np
from mpmath import diff

# 计算基函数，i为控制顶点序号，k为次数，u为代入的值，NodeVector为节点向量
# 该函数返回第i+1个k次基函数在u处的值


def b_spline_basis(i, p, u, nodeVector):
    # nodeVector = np.mat(nodeVector)  # 将输入的节点转化成能够计算的数组
    # k=0时，定义一次基函数
    if p == 0:
        if (nodeVector[i] < u) & (u <= nodeVector[i + 1]):  # 若u在两个节点之间，函数之为1，否则为0
            result = 1
        else:
            result = 0
    else:
        # 计算支撑区间长度
        length1 = nodeVector[i + p] - nodeVector[i]
        length2 = nodeVector[i + p + 1] - nodeVector[i + 1]
        # 定义基函数中的两个系数
        if length1 == 0:  # 特别定义 0/0 = 0
            alpha = 0
        else:
            alpha = (u - nodeVector[i]) / length1  #alpha代表第一项
        if length2 == 0:
            beta = 0
        else:
            beta = (nodeVector[i + p + 1] - u) / length2  #beta代表第二项
    # 递归
        result = alpha * b_spline_basis(i, p - 1, u, nodeVector) + beta * b_spline_basis(i + 1, p - 1, u, nodeVector)
    return result


# 画B样条函数图像
def draw_b_spline(n,p,nodeVector,X,Y):
    plt.figure()
    basis_i = np.zeros(100)  # 存放基函数
    rx = np.zeros(100) 
    ry = np.zeros(100)
    for i in range(n + 1):  # 计算第i个B样条基函数，
        U = np.linspace(nodeVector[0], nodeVector[n+p+1], 100)  # 在节点向量收尾之间取100个点，u在这些点中取值
        j = 0
        for u in U:
            nodeVector = np.array(nodeVector)
            basis_i[j] = b_spline_basis(i, p, u, nodeVector)  # 计算取u时的基函数的值
            j = j + 1
        rx = rx + X[i] * basis_i
        ry = ry + Y[i] * basis_i
        # plt.plot(U,basis_i)
        # print(basis_i)
    # print(rx)
    # print(ry)
    plt.plot(X,Y)
    plt.plot(rx, ry)
    plt.show()



if __name__ == '__main__':
    n = 7
    p= 3

    nodeVector = [0, 0, 0, 0, 0.2, 0.4, 0.6, 0.8, 1, 1, 1, 1]

    ####主要控制量
    X = [0, 1, 2, 3, 4, 5, 6, 7]
    Y = [0, 3, 1, 3, 1, 4, 0, 5]
    draw_b_spline(n,p,nodeVector,X,Y)

