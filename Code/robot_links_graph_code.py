import matplotlib.animation as animation
from matplotlib import pyplot as plt
import numpy as np
from math import sin, cos
import time

L1=89.45
Lr=105.95
L3=100
L4=109
Lm=35
L2=100

phi=np.arctan(100/35) #angle triangle O1-O2-O2prime

plt.rcParams["figure.figsize"] = [14, 10]
plt.rcParams["figure.autolayout"] = True
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

def compute_robot_points(q):

    O0=[0,0,0] # repere 0 en rouge

    O1=[0, 0, L1] #repere 1 en rouge

    O2=[cos(q[0])*(Lr*cos(q[1])), sin(q[0])*(Lr*cos(q[1])), -Lr*sin(q[1]) +L1] #repere 2 en rouge

    O2p=[cos(q[0])*(L2*cos(q[1]-np.pi/2+phi)),  sin(q[0])*(L2*cos(q[1]-np.pi/2+phi)), - L2*sin(q[1]-np.pi/2+phi) +L1] #point O2prime sur le corps 2 en bleu

    O3=[cos(q[0])*(Lr*cos(q[1])+ L3*cos(q[1]+q[2])),  
        sin(q[0])*(Lr*cos(q[1])+ L3*cos(q[1]+q[2])), 
        - Lr*sin(q[1]) - L3*sin(q[1]+q[2])+L1] #repere 3 en rouge

    O4=[cos(q[0])*(Lr*cos(q[1])+ L3*cos(q[1]+q[2])+L4*cos(q[1]+q[2]+q[3])),  
        sin(q[0])*(Lr*cos(q[1])+ L3*cos(q[1]+q[2])+L4*cos(q[1]+q[2]+q[3])), 
        - Lr*sin(q[1]) - L3*sin(q[1]+q[2])-L4*sin(q[1]+q[2]+q[3])+L1] #repere 4 en vert
    
    x = [O0[0], O1[0], O2p[0], O2[0], O3[0], O4[0]]
    y = [O0[1], O1[1], O2p[1], O2[1], O3[1], O4[1]]
    z = [O0[2], O1[2], O2p[2], O2[2], O3[2], O4[2]]
       
    return x, y, z

if __name__=='__main__':

    qi=np.array([0,-phi,phi,0])     #config. initiale (DH) de la figure
    qf=np.array([np.pi/2,-phi,phi-np.pi/2,-np.pi/2]) #config finale (DH)

    # animation
    N=10 # Number of frame
    for i in range(N+1):
       
        q=qi+(qf-qi)*i/N

        x, y, z = compute_robot_points(q)

        ax.set_xlabel('x axis')
        ax.set_ylabel('y axis')
        ax.set_zlabel('z axis')
        ax.scatter(x, y, z, c=['red', 'red', 'blue', 'red', 'red', 'green'], s=20) #points sur les liaisons
        ax.plot(x, y, z) #tracer des lignes entres les points
        plt.pause(0.1) #pause avec duree en secondes

    plt.show()

 

    