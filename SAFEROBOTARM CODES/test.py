#THIS PROGRAM DEFINE THE INVERSE KINEMATICS OF OF  6 DOF ROBOTARM 
# TELL HOW TO MOVE A ROBOT ARM TO A POINT IN SAPCE CALCULATING THE ANGLE ITS SELF
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


pose=[0.4, 0, 0.2, 0, 0, 0]
X,Y,Z,roll,pitch,yaw = pose
d1=0.1625
d6=0.0823
a2=0.425
a3=0.3922
d4=0.1333
d5=0.0997
d6=0.0996
#print(X)
base_x,base_y=0,0
Yaw_matrix=np.array([[np.cos(yaw),-np.sin(yaw),0],
                     [np.sin(yaw),np.cos(yaw),0],
                     [0,0,1]]) 
Pitch_matrix=np.array([[np.cos(pitch),0,np.sin(pitch)],
                       [0,1,0],
                       [-np.sin(pitch),0,np.cos(pitch)]])
Roll_matrix=np.array([[1,0,0],
                      [0,np.cos(roll),-np.sin(roll)],
                      [0,np.sin(roll),np.cos(roll)]])

Rotation_matrix_0_6=Yaw_matrix@Pitch_matrix@Roll_matrix
print(Rotation_matrix_0_6)
r13,r23,r33=Rotation_matrix_0_6[:,2]

WC_x=X-d6*r13
WC_y=Y-d6*r23
WC_z=Z-d6*r33

Theta_1=math.atan2(WC_y,WC_x)

R=math.sqrt(WC_x**2+WC_y**2)
S=WC_z-d1

C=math.sqrt(R**2+S**2)

Phi_2=math.atan2(S,R)
inside_1=(a2**2+C**2-a3**2)/(2*C*a2)
Phi_1=math.acos(inside_1)

Theta_2=Phi_2-Phi_1

inside_2=(a2**2+a3**2-C**2)/(2*a2*a3)
Phi_3=math.acos(inside_2)

Theta_3=math.pi-Phi_3

Rot_matrix_0_1=np.array([[np.cos(Theta_1),0,np.sin(Theta_1)],
                         [np.sin(Theta_1),0,-np.cos(Theta_1)],
                         [0,1,0]])

Rot_matrix_1_2=np.array([[np.cos(Theta_2),-np.sin(Theta_2),0],
                         [np.sin(Theta_2),np.cos(Theta_2),0],
                         [0,0,1]])

Rot_matrix_2_3=np.array([[np.cos(Theta_3),0,np.sin(Theta_3)],
                         [np.sin(Theta_3),0,-np.cos(Theta_3)],
                         [0,1,0]])


Rotation_matrix_0_3=Rot_matrix_0_1@Rot_matrix_1_2@Rot_matrix_2_3
Rotation_matrix_0_3_inverse=Rotation_matrix_0_3.T
Rotation_matrix_3_6=Rotation_matrix_0_3_inverse@Rotation_matrix_0_6

Theta_4=math.atan2(Rotation_matrix_3_6[1,2],Rotation_matrix_3_6[0,2])
Theta_5=math.acos(Rotation_matrix_3_6[2,2])
Theta_6=math.atan2(Rotation_matrix_3_6[2,1],-Rotation_matrix_3_6[2,0])

Rot_matrix_3_4=np.array([[np.cos(Theta_4),0,-np.sin(Theta_4)],
                         [np.sin(Theta_4),0,np.cos(Theta_4)],
                         [0,-1,0]])

Rot_matrix_4_5=np.array([[np.cos(Theta_5),0,np.sin(Theta_5)],
                         [np.sin(Theta_5),0,-np.cos(Theta_5)],
                         [0,1,0]])
Rot_matrix_5_6=np.array([[np.cos(Theta_6),-np.sin(Theta_6),0],
                         [np.sin(Theta_6),np.cos(Theta_6),0],
                         [0,0,1]])

print(Theta_1,Theta_2,Theta_3,Theta_4,Theta_5,Theta_6)


# transformation matrix
T_0_1=np.array([[np.cos(Theta_1),0,np.sin(Theta_1),0],
                [np.sin(Theta_1),0,-np.cos(Theta_1),0],
                [0,1,0,d1],
                [0,0,0,1]])

T_1_2=np.array([[np.cos(Theta_2),-np.sin(Theta_2),0,a2*np.cos(Theta_2)],
                [np.sin(Theta_2),np.cos(Theta_2),0,a2*np.sin(Theta_2)],
                [0,0,1,0],
                [0,0,0,1]])

T_2_3=np.array([[np.cos(Theta_3),0,np.sin(Theta_3),a3*np.cos(Theta_3)],
                [np.sin(Theta_3),0,-np.cos(Theta_3),a3*np.sin(Theta_3)],
                [0,1,0,0],
                [0,0,0,1]])

T_3_4=np.array([[np.cos(Theta_4),0,-np.sin(Theta_4),0],
                [np.sin(Theta_4),0,np.cos(Theta_4),0],
                [0,-1,0,d4],
                [0,0,0,1]])

T_4_5=np.array([[np.cos(Theta_5),0,np.sin(Theta_5),0],
                [np.sin(Theta_5),0,-np.cos(Theta_5),0],
                [0,1,0,d5],
                [0,0,0,1]])

T_5_6=np.array([[np.cos(Theta_6),-np.sin(Theta_6),0,0],
                [np.sin(Theta_6),np.cos(Theta_6),0,0],
                [0,0,1,d6],
                [0,0,0,1]])

T_0_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6

T_0_2 = T_0_1 @ T_1_2
T_0_3 = T_0_2 @ T_2_3
T_0_4 = T_0_3 @ T_3_4
T_0_5 = T_0_4 @ T_4_5
T_0_6 = T_0_5 @ T_5_6

p_0 = [0,0,0]
p_1 = T_0_1[:3,3]
p_2 = T_0_2[:3,3]
p_3 = T_0_3[:3,3]
p_4 = T_0_4[:3,3]
p_5 = T_0_5[:3,3]
p_6 = T_0_6[:3,3]


# ================== ANIMATION PART ==================

def get_points(T1, T2, T3, T4, T5, T6):
    T01 = T1
    T02 = T01 @ T2
    T03 = T02 @ T3
    T04 = T03 @ T4
    T05 = T04 @ T5
    T06 = T05 @ T6

    return np.array([
        [0,0,0],
        T01[:3,3],
        T02[:3,3],
        T03[:3,3],
        T04[:3,3],
        T05[:3,3],
        T06[:3,3]
    ])


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(0,1)

line, = ax.plot([], [], [], 'o-', linewidth=4)

steps = 100

theta1_vals = np.linspace(0, Theta_1, steps)
theta2_vals = np.linspace(0, Theta_2, steps)
theta3_vals = np.linspace(0, Theta_3, steps)
theta4_vals = np.linspace(0, Theta_4, steps)
theta5_vals = np.linspace(0, Theta_5, steps)
theta6_vals = np.linspace(0, Theta_6, steps)


def update(i):
    t1 = theta1_vals[i]
    t2 = theta2_vals[i]
    t3 = theta3_vals[i]
    t4 = theta4_vals[i]
    t5 = theta5_vals[i]
    t6 = theta6_vals[i]

    T1 = np.array([[np.cos(t1),0,np.sin(t1),0],
                   [np.sin(t1),0,-np.cos(t1),0],
                   [0,1,0,d1],
                   [0,0,0,1]])

    T2 = np.array([[np.cos(t2),-np.sin(t2),0,a2*np.cos(t2)],
                   [np.sin(t2),np.cos(t2),0,a2*np.sin(t2)],
                   [0,0,1,0],
                   [0,0,0,1]])

    T3 = np.array([[np.cos(t3),0,np.sin(t3),a3*np.cos(t3)],
                   [np.sin(t3),0,-np.cos(t3),a3*np.sin(t3)],
                   [0,1,0,0],
                   [0,0,0,1]])

    T4 = np.array([[np.cos(t4),0,-np.sin(t4),0],
                   [np.sin(t4),0,np.cos(t4),0],
                   [0,-1,0,d4],
                   [0,0,0,1]])

    T5 = np.array([[np.cos(t5),0,np.sin(t5),0],
                   [np.sin(t5),0,-np.cos(t5),0],
                   [0,1,0,d5],
                   [0,0,0,1]])

    T6 = np.array([[np.cos(t6),-np.sin(t6),0,0],
                   [np.sin(t6),np.cos(t6),0,0],
                   [0,0,1,d6],
                   [0,0,0,1]])

    pts = get_points(T1, T2, T3, T4, T5, T6)

    line.set_data(pts[:,0], pts[:,1])
    line.set_3d_properties(pts[:,2])

    return line,


ani = FuncAnimation(fig, update, frames=steps, interval=50)

plt.show()