'''A simple example script, which implements an LQR controller for a double integrator.
'''

from __future__ import print_function, division

import controlpy

import numpy as np

# Example system is a double integrator:
A=np.array([[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0],
                        [0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0],[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]])
B=np.array([[0,0,0 ,0 ,0 ,0 ,0 ,0],[0 ,0 ,1 ,0 ,0 ,0 ,0 ,0]])
Q1=np.array([[1,0,0,-1,0,0,0,0],[0,1,0,0,0,-1,0,0],[0,0,1,0,0,0,0,-1]])
Q2=Q1.transpose()

I3=np.eye(3,dtype=int)
Q3=np.dot(Q2,I3)
Q=np.dot(Q3,Q1)
A=A-0.05*np.eye(8,dtype=int)
R=0.0001*np.eye(2,dtype=int)
K,S,E = controlpy.synthesis.controller_lqr(A, B.transpose(), Q, R)

print('The computed gain is:')
print(K)
x=np.zeros((1,5))
x[0,1]=5
print(x)