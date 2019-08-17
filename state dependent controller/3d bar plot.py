from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure()
ax1 = fig.add_subplot(111, projection='3d')
'''x=[]
for i in range(0,15):
    x.append(i)
y=[]
for i in range(0,6):
    y.append(i)
X_bar=[]
Y_bar=[]
for i in x:
    for j in y:
        X_bar.append(i)
        Y_bar.append(j)
print(X_bar)
print(Y_bar)'''
x=[0,60,120,180,240,300,360,420,480,540,600,660,720,780,840]
y=[0,60,120,180,240,300]
X_bar=[]
Y_bar=[]
for i in x:
    for j in y:
        X_bar.append(i)
        Y_bar.append(j)
print(X_bar)
print(Y_bar)
Z_bar=np.zeros(90)


'''x3 = [1,2,3,4,5,6,7,8,9,10]
y3 = [5,6,7,8,2,5,6,3,7,2]
z3 = np.zeros(10)'''

dx = np.ones(90)
for i in range(0,len(dx)):
    dx[i]=20
dy = np.ones(90)
for i in range(0,len(dy)):
    dy[i]=20

dz = np.ones(90)
for i in range(0,len(dz)):
    dz[i]=1
dz[14]=dz[13]=dz[15]=dz[20]=dz[8]=dz[9]=dz[19]=dz[21]=dz[7]=dz[2]=dz[12]=dz[16]=dz[26]=0
dz[69]=dz[63]=dz[68]=dz[70]=dz[75]=dz[74]=dz[76]=dz[64]=dz[62]=dz[67]=dz[57]=dz[71]=dz[81]=0
ax1.bar3d(X_bar, Y_bar, Z_bar, dx, dy, dz,alpha=1,color='#F0F0F0')

X_bar4=[120,120,0,240,660,660,540,780]
Y_bar4=[0,240,120,120,60,300,180,180]
Z_bar4=[0,0,0,0,0,0,0,0]
dz4=5
ax1.bar3d(X_bar4, Y_bar4, Z_bar4, dx, dy, dz4,alpha=1,color='#C0C0C0')

X_bar3=[60,180,60,180,600,720,600,720]
Y_bar3=[60,60,180,180,120,120,240,240]
Z_bar3=[0,0,0,0,0,0,0,0]
dz3=10
ax1.bar3d(X_bar3, Y_bar3, Z_bar3, dx, dy, dz3,alpha=1,color='#A9A9A9')

X_bar2=[120,120,60,180,660,660,600,720]
Y_bar2=[60,180,120,120,120,240,180,180]
Z_bar2=[0,0,0,0,0,0,0,0]
dz2=15
ax1.bar3d(X_bar2, Y_bar2, Z_bar2, dx, dy, dz2,alpha=1,color='#696969')

X_bar1=[120,660]
Y_bar1=[120,180]
Z_bar1=[0,0]
dz1=20
ax1.bar3d(X_bar1, Y_bar1, Z_bar1, dx, dy, dz1,color='#2B2B2B',alpha=0.05)
'''dz[14]=20
dz[13]=dz[15]=dz[20]=dz[8]=15
dz[9]=dz[19]=dz[21]=dz[7]=10
dz[2]=dz[12]=dz[16]=dz[26]=5
dz[69]=20
dz[63]=dz[68]=dz[70]=dz[75]=15
dz[74]=dz[76]=dz[64]=dz[62]=10
dz[67]=dz[57]=dz[71]=dz[81]=5'''

#ax1.bar3d(x3, y3, z3, dx, dy, dz)
#ax1.bar3d(X_bar, Y_bar, Z_bar, dx, dy, dz,color='dimgray')
ax1.view_init(55,-81)
ax1.set_xlabel('x axis')
ax1.set_ylabel('y axis')
ax1.set_zlabel('Priority value')

plt.show()
'''from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

#from IPython.Shell import IPShellEmbed
#sh = IPShellEmbed()

data = np.array([
[0,1,0,2,0],
[0,3,0,2,0],
[6,1,1,7,0],
[0,5,0,2,9],
[0,1,0,4,0],
[9,1,3,4,2],
[0,0,2,1,3],
])

column_names = ['a','b','c','d','e']
row_names = ['Mon','Tue','Wed','Thu','Fri','Sat','Sun']

fig = plt.figure()
ax = Axes3D(fig)

lx= len(data[0])            # Work out matrix dimensions
ly= len(data[:,0])
xpos = np.arange(0,lx,1)    # Set up a mesh of positions
ypos = np.arange(0,ly,1)
xpos, ypos = np.meshgrid(xpos+0.25, ypos+0.25)

xpos = xpos.flatten()   # Convert positions to 1D array
ypos = ypos.flatten()
zpos = np.zeros(lx*ly)

dx = 0.5 * np.ones_like(zpos)
dy = dx.copy()
dz = data.flatten()

ax.bar3d(xpos,ypos,zpos, dx, dy, dz, color='b')

#sh()
ax.w_xaxis.set_ticklabels(column_names)
ax.w_yaxis.set_ticklabels(row_names)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('Priority value')
plt.show()'''
