import numpy as np
import bisect
import matplotlib.pyplot as plt

class Spline:

    def __init__(self, x,y):

        self.x=x
        self.y=y
        #点集数
        self.n=len(x)
        #计算h值
        h=np.diff(x)

        self.a=[]
        self.b =[]
        self.c =[]
        self.d = []

        self.a=[i for i in y]
     #   print(self.a)
        A=self.A_calc(h)
     #   print(A)
        B=self.B_calc(h)
     #   print(B)

        self.c=np.linalg.solve(A, B)
    #    print(self.c)

        for i in range(self.n-1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            self.b.append((self.a[i + 1] - self.a[i]) / h[i] - h[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0)



    #    print(self.d)
    #    print(self.b)

    #计算公式的左侧矩阵
    def A_calc(self,h):
        A = np.zeros((self.n, self.n))
        A[0, 0] = 1.0
        for i in range(self.n-2):
            A[i+1,i+1] = 2.0 * (h[i] + h[i + 1])
            A[i+1,i]=h[i]
            A[i + 1, i+2] = h[i+1]
        A[self.n-1, self.n-1]=1.0

        return A

    #计算公式的右侧矩阵
    def B_calc(self,h):
        B=np.zeros((self.n,1))
        for i in range(self.n - 2):
            B[i+1]=3.0*((self.a[i+2]-self.a[i + 1])/ h[i + 1]-(self.a[i + 1] - self.a[i]) / h[i])
        B[0]=0
        B[self.n - 1]=0
        return B

   ##不一样的地方**
    def calc_equation(self,acc,tx):
        ti=bisect.bisect(self.x, tx) - 1
        dx=tx - self.x[acc]
        result = self.a[acc] + self.b[acc] * dx + self.c[acc] * dx ** 2.0 + self.d[acc] * dx ** 3.0
        return result
def main():

    x=[-8, -6, -4, -2, 0, 2, 4, 6, 8, 10,14,16,13,10,8,6,4,2,0,-2,-4,-6,-8]
    y=[0.8, 0.5, 0.1, 1.2, 1.9, 2.0, 3.0,  1.5,1.2,0.6,4.5,8.6,10.8,11.2,12.3,13.9,14.5,12.3,11.8,10.4,13.4,15.6,11.1]

     # 初始化散点
    # x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    # y = [0.7,   -6,   5, 6.5, 0.0, 5.0, -2.0]


    spline = Spline(x, y)
    rx=[]
    ry=[]
    res=[]
    ro=[]
 
    #判断xi和xi+1的大小
    for i in range(len(x)-1):
        if x[i]<x[i+1]:
            ro=np.arange(x[i], x[i+1], 0.01)
        if x[i]>x[i+1]:
            ro=np.arange(x[i], x[i + 1], -0.01)
        account=i
        res=[spline.calc_equation(account,i) for i in ro]
        ry.extend(res)
        rx.extend(ro)
        if i==1:
            print(len(ry))
            print(len(rx))

    plt.plot(x, y, "ob", label='Point set')
    plt.plot(rx, ry, "-r",label='Curve fitting')
    plt.axis("equal")
    plt.legend()
    plt.show()


if __name__ == '__main__':
		main()