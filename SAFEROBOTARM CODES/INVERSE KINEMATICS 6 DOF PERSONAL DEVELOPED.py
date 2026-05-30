#THIS PROGRAM DEFINE THE INVERSE KINEMATICS OF OF  6 DOF ROBOTARM 
# TELL HOW TO MOVE A ROBOT ARM TO A POINT IN SAPCE CALCULATING THE ANGLE ITS SELF
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


pose=[0.4, 0, 0.2, 0, 0, 0]
X,Y,Z,roll,pitch,yaw = pose
d1=0.1625
a2=-0.425
a3=-0.3922
d4=0.1333
d5=0.0997
d6=0.0996


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
r13 = Rotation_matrix_0_6[0, 2]
r23 = Rotation_matrix_0_6[1, 2]
r33 = Rotation_matrix_0_6[2, 2]
#print(r13,r23,r33)


WC_x = X - (d6 * r13)
WC_y = Y - (d6 * r23)
WC_z = Z - (d6 * r33)

# --- 2. Theta 1 with Shoulder Offset (d4) ---
# This accounts for the fact that the UR5e shoulder is shifted sideways
R_total = math.sqrt(WC_x**2 + WC_y**2)
if R_total < abs(d4):
    raise ValueError("Target out of reach (inside shoulder offset deadzone)")

# Base angle to WC + correction for the d4 offset triangle
alpha = math.atan2(WC_y, WC_x)
beta = math.acos(d4 / R_total)
# For 'Shoulder Left' configuration:
Theta_1 = alpha + beta + math.pi/2 

# --- 3. Solving the 2D Triangle for Theta 2 & 3 ---
# We must use the "Planar Reach" (R_planar) from the shoulder pivot, not the base center
R_planar = math.sqrt(R_total**2 - d4**2)
S = WC_z - d1
C = math.sqrt(R_planar**2 + S**2)

# Angle to the Wrist Center relative to the Shoulder Joint
Phi_2 = math.atan2(S, R_planar)

# Law of Cosines to find internal triangle angles
inside_1 = (a2**2 + C**2 - a3**2) / (2 * C * a2)
Phi_1 = math.acos(np.clip(inside_1, -1, 1)) # np.clip prevents math domain errors
Theta_2 = Phi_2 - Phi_1

inside_2 = (a2**2 + a3**2 - C**2) / (2 * a2 * a3)
Phi_3 = math.acos(np.clip(inside_2, -1, 1))
Theta_3 = math.pi - Phi_3

#servo_0_angle= 302
#servo_1_angle= 190
#servo_2_angle= 59
#servo_3_angle= 199
#servo_4_angle= 120
#servo_5_angle= 90


#Theta_1=np.deg2rad(servo_0_angle)
#Theta_2=np.deg2rad(servo_1_angle)
#Theta_3=np.deg2rad(servo_2_angle)
#Theta_4=np.deg2rad(servo_3_angle)
#Theta_5=np.deg2rad(servo_4_angle)
#Theta_6=np.deg2rad(servo_5_angle)



#print(Theta_6)

Rot_matrix_0_1=np.array([[np.cos(Theta_1), -np.sin(Theta_1)*np.cos(np.pi/2),  np.sin(Theta_1)*np.sin(np.pi/2)],
                         [np.sin(Theta_1),  np.cos(Theta_1)*np.cos(np.pi/2), -np.cos(Theta_1)*np.sin(np.pi/2)],
                         [0,np.sin(np.pi/2), np.cos(np.pi/2)]])


Rot_matrix_1_2=np.array([[np.cos(Theta_2), -np.sin(Theta_2)*np.cos(0),  np.sin(Theta_2)*np.sin(0)],
                         [np.sin(Theta_2),  np.cos(Theta_2)*np.cos(0), -np.cos(Theta_2)*np.sin(0)],
                         [0,np.sin(0), np.cos(0)]])


Rot_matrix_2_3=np.array([[np.cos(Theta_3), -np.sin(Theta_3)*np.cos(0),  np.sin(Theta_3)*np.sin(0)],
                         [np.sin(Theta_3),  np.cos(Theta_3)*np.cos(0), -np.cos(Theta_3)*np.sin(0)],
                         [0,np.sin(0), np.cos(0)]])

print(Rot_matrix_2_3)

Rotation_matrix_0_3=Rot_matrix_0_1@Rot_matrix_1_2@Rot_matrix_2_3
Rotation_matrix_0_3_inverse=Rotation_matrix_0_3.T
Rotation_matrix_3_6=Rotation_matrix_0_3_inverse@Rotation_matrix_0_6

Theta_4=math.atan2(Rotation_matrix_3_6[0,2],-Rotation_matrix_3_6[1,2])

# ✅ FIX: safe acos

Theta_5=math.acos(np.clip(Rotation_matrix_3_6[1,2], -1, 1))

Theta_6=math.atan2(-Rotation_matrix_3_6[2,0],Rotation_matrix_3_6[2,1])

#print(math.pi/2)

def DH(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

T_0_1 = DH(Theta_1, 0, d1,  np.pi/2)
T_1_2 = DH(Theta_2, a2, 0,  0)
T_2_3 = DH(Theta_3, a3, 0,  0)
T_3_4 = DH(Theta_4, 0, d4,  np.pi/2)
T_4_5 = DH(Theta_5, 0, d5, -np.pi/2)
T_5_6 = DH(Theta_6, 0, d6,  0)


T_0_6=T_0_1@T_1_2@T_2_3@T_3_4@T_4_5@T_5_6





#print("WC_x, WC_y, WC_z:", WC_x, WC_y, WC_z)

#print("Theta_1, Theta_2, Theta_3 (deg):", np.degrees([Theta_1, Theta_2, Theta_3]))


#print("R_0_3:\n", Rotation_matrix_0_3)


Target_Pos = np.array([0.4, 0, 0.2]) 

# Calculated Position from Forward Kinematics
Calculated_Pos = T_0_6[0:3, 3]


#print(T_4_5)
print(T_0_1@T_1_2@T_2_3@T_3_4@T_4_5@T_5_6)
#print(T_2_3)
error = np.linalg.norm(Target_Pos - Calculated_Pos)
#P_EE = T_0_6[:3, 3]  # first 3 elements of last column
#print("End Effector Position (x, y, z):", P_EE)

print(f"Calculated XYZ: {Calculated_Pos}")
print(f"Error Magnitude: {error:.6f} meters")

if error < 0.001:
    print("✅ Success: IK matches FK within 1mm!")
else:
    print("❌ Error: Check your theta offsets or d4/d5 values.")

# Extract end effector orientation (rotation matrix)
#R_EE = T_0_6[:3, :3]  # top-left 3x3 submatrix
#print("End Effector Rotation Matrix:\n", R_EE)