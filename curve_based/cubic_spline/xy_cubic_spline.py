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
    def calc_equation(self, tx):
        ti = bisect.bisect(self.x, tx) - 1
        dx = tx - self.x[ti]
        result = self.a[ti] + self.b[ti] * dx + self.c[ti] * dx ** 2.0 + self.d[ti] * dx ** 3.0

        return result

def main():

    x = [-8, -3, 0.0, 6, 8, 10, 16, 19]
    y = [4.5, 0.8, 9.6, 8.8, 6.5, 4.5, -3.0, -8]


    spline = Spline(x, y)

    rx = np.arange(-8, 19, 0.01)

    ry = [spline.calc_equation(i) for i in rx]

    plt.plot(x, y, "ob", label='Point set')
    plt.plot(rx, ry, "-r",label='Curve fitting')
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()


if __name__ == '__main__':
		main()