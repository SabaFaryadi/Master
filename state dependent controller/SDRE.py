from collections import defaultdict, deque
import numpy as np
import matplotlib.pyplot as plt
import math
import controlpy
import os

# Generating plots
def CordinateGenerator(w,l,seg):
    # distance between points
    a = 100
    b = 100
    c = 100 / 3
    d = 100 / 3
    e = 100 / 3
    f = 100 / 3

    # X,Y coordinates. Set to a size to accomidate p the total number of points
    p = (w + 1) * (l + 1) + w * l * (seg - 1)
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    # Loops through every point. Rx and Ry are the running x and y positions
    # n=0 is assumed to be the origin
    n = 0
    Rx = 0
    Ry = 0
    for i in range(0, l):
        x[n] = Rx
        y[n] = Ry
        n = n + 1
        for j in range(0, w):
            Rx = Rx + c
            Ry = Ry + d
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx + e
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry = Ry + f
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx - e
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Rx = Rx - c
            Ry = Ry + d
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Ry = 0
        Rx = Rx + a

    # clean up for the last row since the numbering pattern breaks
    Ry = 0
    for i in range(0, w + 1):
        x[n] = Rx
        y[n] = Ry
        Ry = Ry + b
        n = n + 1
    return x,y


def SDRE(X_path,Y_path):
    delta_t = 0.01
    t = 10
    x_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    y_initial = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    tetha = np.zeros((len(X_path)-1, (int(t / delta_t)) - 1))
    zd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    zd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    wd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd1 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    yd2 = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    v = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    w = np.zeros((len(X_path) - 1, (int(t / delta_t)) - 1))
    for j in range(0,len(X_path)-1):
        x=X_path[j]
        y=Y_path[j]
        x_final = X_path[j + 1]
        y_final = Y_path[j + 1]

        if x!=x_final or y!=y_final:
            if x==x_final:
                x_final=x_final+0.01

            x_initial[j, 0] = x
            y_initial[j, 0] = y

            tetha[j,0]=math.pi/2

            zd1[j,0]=x_initial[j,0]
            zd2[j,0]=(x_final-x_initial[j,0])/t





            m=(y_final-y_initial[j,0])/(x_final-x_initial[j,0])
            n=y_initial[j,0]-m*x_initial[j,0]


            wd1[j,0]=math.atan(m)


            yd1[j,0]=(m*x_initial[j,0])+n
            yd2[j,0]=m*(x_final-x_initial[j,0])/t

            Q1=np.array([[1,0,0,-1,0,0,0,0],[0,1,0,0,0,-1,0,0],[0,0,1,0,0,0,0,-1]])
            Q2=Q1.transpose()
            I3=np.eye(3,dtype=int)
            Q3=np.dot(Q2,I3)
            Q=np.dot(Q3,Q1)

            for time in range(0,(int(t/delta_t))-2):
                A=np.array([[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0],
                            [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]])
                B=np.array([[math.cos(tetha[j,time]),math.sin(tetha[j,time]),0 ,0 ,0 ,0 ,0 ,0],[0 ,0 ,1 ,0 ,0 ,0 ,0 ,0]])
                A = A - 0.05 * np.eye(8, dtype=int)
                R = 0.001 * np.eye(2, dtype=int)
                K,S,E = controlpy.synthesis.controller_lqr(A, B.transpose(), Q, R)
                #print((K))
                u=np.array([x_initial[j,time],y_initial[j,time],tetha[j,time],zd1[j,time],zd2[j,time],yd1[j,time],yd2[j,time],wd1[j,time]])
                #print(type(u))
                U=np.dot(-K,u.transpose())

                v[j,time]=U[0]
                w[j,time]=U[1]

                x_initial[j,time+1]=x_initial[j,time]+delta_t*(U[0]*math.cos(tetha[j,time]))
                y_initial[j,time+1]=y_initial[j,time]+delta_t*(U[0]*math.sin(tetha[j,time]))
                tetha[j,time+1]=tetha[j,time]+delta_t*U[1]
                zd1[j,time+1]=zd1[j,time]+delta_t*zd2[j,time]
                zd2[j,time+1]=zd2[j,time]
                yd1[j,time+1]=yd1[j,time]+delta_t*yd2[j,time]
                yd2[j,time+1]=yd2[j,time]
                wd1[j,time+1]=wd1[j,time]
    return x_initial,y_initial,v,w
if __name__ == '__main__':

    x,y=CordinateGenerator(3,4,5)
    plt.scatter(x, y)
    #plt.show()
    Xpath=[0,0]
    Ypath=[0,100]

    trajectory=SDRE(Xpath,Ypath)
    print(trajectory[2][0])
    print (trajectory[3][0])
    #print(trajectory[1])
    plt.scatter(trajectory[0],trajectory[1])
    plt.show()