import numpy as np
import matplotlib.pyplot as plt
import math

def CordinateGenerator(w,l,seg):
    a=110
    b=70
    c=60
    d=60
    p=w*l*seg
    x = np.zeros(p, dtype=float)
    y = np.zeros(p, dtype=float)

    Rx=0
    Ry=0
    n=0
    for i in range(0,w):
        print('i=',i)

        for j in range(0,l):
            if j==0:
                Rx=Rx
            else:
                Rx=Rx+(d)
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry=Ry+b
            x[n]=Rx
            y[n]=Ry
            n=n+1

            Rx=Rx+a
            x[n] = Rx
            y[n] = Ry
            n = n + 1

            Ry=Ry-b
            x[n] = Rx
            y[n] = Ry
            n = n + 1
        Rx=0
        Ry=(i+1)*(b+c)
    return x,y

if __name__ == '__main__':
    X,Y=CordinateGenerator(3,4,4)
    plt.scatter(X,Y)
    i=0
    for p in range(0,len(X)):
        plt.annotate(i, xy=(X[p], Y[p]), color='k', size=10)
        i=i+1
    print(p)
    plt.show()