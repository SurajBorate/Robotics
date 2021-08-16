"""
Verify IK solution by FK for 3DOF Articulated Arm
Book reference: Vidyasagar,Spong - Robot Dynamics and Control
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def c(theta):
    return math.cos(theta)

def s(theta):
    return math.sin(theta)

def homogenous_matrix(DH1):
    ct = math.cos(DH1[0])
    ca = math.cos(DH1[1])
    st = math.sin(DH1[0])
    sa = math.sin(DH1[1])
    a = DH1[1]
    d = DH1[2]
    r = DH1[3]
    homogenous_matrix =np.array([[ct, -st*ca,  st*sa,   r*ct],
                                [st,   ct*ca,  -ct*sa,  r*st],
                                [0,    sa,     ca,         d],
                                [0,    0,      0,          1]])
    return homogenous_matrix

def forward_kinematics(DH):
    A = np.identity(4)
    n = np.shape(DH)[0]
    ox = [0]
    oy = [0]
    oz = [0]
    for i in range(n):
        Anext = homogenous_matrix(DH[i])
        A = np.matmul(A,Anext)

        ox.append(A[0][3])
        oy.append(A[1][3])
        oz.append(A[2][3])
    return ox,oy,oz


def inverse_kinematics(l1,l2,l3,xc,yc,zc):
    theta1 = np.rad2deg(math.atan2(yc,xc))
    D = (xc*xc+yc*yc+(zc-l1)*(zc-l1)-l2*l2-l3*l3)/(2*l2*l3)
    if D>=1 or D<=-1:
        print("singular configuration")
    if D>1 or D<-1:
        print("outside workspace")
    theta3 = (math.atan2((-math.sqrt(1-D*D)),D))
    theta2 = np.rad2deg(math.atan2(zc-l1,(math.sqrt(xc*xc+yc*yc)))-math.atan2((l3*math.sin(theta3)),(l2+l3*math.cos(theta3))))
    theta3 = np.rad2deg(theta3)
    return theta1,theta2,theta3
#"Demand endpoint"
xc = 1
yc = 1
zc = 1

print('demanded endpoint',xc,yc,zc)

#"Link lengths"
l1 =0.5
l2 =1
l3 =0.5

#Get joint angles from end effector position: IK

t1,t2,t3=inverse_kinematics(l1,l2,l3,xc,yc,zc)
print('joint angle by IK',t1,t2,t3)

"converting theta to radian"
theta1 = np.deg2rad(t1)
theta2 = np.deg2rad(t2)
theta3 = np.deg2rad(t3)

"DH matrix for 3 DOF Articulated arm"
DH = [[theta1,-np.deg2rad(90),l1,0],[theta2,0,0,l2],[theta3,0,0,l3]]

"Use DH for FK to get endpoint position"
ox,oy,oz = forward_kinematics(DH)

link1x = [ox[0],ox[1]]
link1y = [oy[0],oy[1]]
link1z = [oz[0],oz[1]]

link2x = [ox[1],ox[2]]
link2y = [oy[1],oy[2]]
link2z = [oz[1],oz[2]]

link3x = [ox[2],ox[3]]
link3y = [oy[2],oy[3]]
link3z = [oz[2],oz[3]]

linkx = [link1x,link2x,link3x]
linky = [link1y,link2y,link3y]
linkz = [link1z,link2z,link3z]

ax = plt.axes(projection='3d')

#ax.plot3D(ox,oy,oz)
# ax.plot3D(link1x,link1y,link1z)
# ax.plot3D(link2x,link2y,link2z)
# ax.plot3D(link3x,link3y,link3z)

ax.set_zlabel('z')
ax.set_xlabel('x')
ax.set_ylabel('y')

elev = 45
azim = 45
ax.view_init(elev, azim)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 1)

print('verification of endpoint by FK',ox[3],oy[3],oz[3])

tp = np.linspace(0,1000,1000)
xp = np.cos(tp)
xp = xp.tolist()
yp = np.sin(tp)
yp = yp.tolist()
zp = 0.5*np.ones([np.shape(tp)[0]])
zp = zp.tolist()
ax.plot3D(xp,yp,zp,'-g',linewidth=0.01)
plt.pause(0.1)


for i in range(0,np.shape(tp)[0]):
    tp1= tp[i]
    xc = math.cos(tp[i])
    yc = math.sin(tp[i])
    zc = 0.5

    t1,t2,t3=inverse_kinematics(l1, l2, l3, xc, yc, zc)

    "converting theta to radian"
    theta1 = np.deg2rad(t1)
    theta2 = np.deg2rad(t2)
    theta3 = np.deg2rad(t3)

    "DH matrix for 3 DOF Articulated arm"
    DH = [[theta1, np.deg2rad(90), l1, 0], [theta2, 0, 0, l2], [theta3, 0, 0, l3]]

    "Use DH for FK to get endpoint position"
    ox, oy, oz = forward_kinematics(DH)

    link1x = [ox[0], ox[1]]
    link1y = [oy[0], oy[1]]
    link1z = [oz[0], oz[1]]

    link2x = [ox[1], ox[2]]
    link2y = [oy[1], oy[2]]
    link2z = [oz[1], oz[2]]

    link3x = [ox[2], ox[3]]
    link3y = [oy[2], oy[3]]
    link3z = [oz[2], oz[3]]

    linkx = [link1x, link2x, link3x]
    linky = [link1y, link2y, link3y]
    linkz = [link1z, link2z, link3z]

    #ax = plt.axes(projection='3d')

    # ax.plot3D(ox,oy,oz)
    elev = 45
    azim = 45
    ax.view_init(elev, azim)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(0, 1)
    a =ax.plot3D(link1x, link1y, link1z,'-r')
    b =ax.plot3D(link2x, link2y, link2z,'-b')
    c =ax.plot3D(link3x, link3y, link3z,'-k')
    plt.pause(0.1)
    for handle in a:
        handle.remove()

    for handle in b:
        handle.remove()

    for handle in c:
        handle.remove()


plt.show()
