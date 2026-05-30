# UR5e Inverse Kinematics + 3D Animation
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---- Target Pose ----
pose = [0.4, 0, 0, 0, np.pi, 0]
X, Y, Z, roll, pitch, yaw = pose

# ---- UR5e DH Parameters ----
d1 =  0.1625
a2 = -0.425
a3 = -0.3922
d4 =  0.1333
d5 =  0.0997
d6 =  0.0996

ZERO_THRESH = 1e-8

# ---- Rotation Matrix ----
Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
               [math.sin(yaw),  math.cos(yaw), 0],
               [0,              0,             1]])

Ry = np.array([[ math.cos(pitch), 0, math.sin(pitch)],
               [ 0,               1, 0              ],
               [-math.sin(pitch), 0, math.cos(pitch)]])

Rx = np.array([[1, 0,             0            ],
               [0, math.cos(roll), -math.sin(roll)],
               [0, math.sin(roll),  math.cos(roll)]])

R = Rz @ Ry @ Rx

T = np.eye(4)
T[:3, :3] = R
T[:3,  3] = [X, Y, Z]

# ---- Extract ----
T00,T01,T02,T03 = T[0]
T10,T11,T12,T13 = T[1]
T20,T21,T22,T23 = T[2]

# ---- Theta 1 ----
A = d6 * T12 - T13
B = d6 * T02 - T03
R_sq = A*A + B*B

arccos_val = math.acos(np.clip(d4 / math.sqrt(R_sq), -1, 1))
arctan_val = math.atan2(-B, A)

Theta_1 = arccos_val + arctan_val






# ---- Theta 5 ----
c1, s1 = math.cos(Theta_1), math.sin(Theta_1)
numer = T03 * s1 - T13 * c1 - d4
Theta_5 = math.acos(np.clip(numer / d6, -1, 1))




# ---- Theta 6 ----
s5 = math.sin(Theta_5)

Theta_6 = math.atan2(
        -(T01 * s1 - T11 * c1),
         (T00 * s1 - T10 * c1)
    )




# ---- Theta 2,3,4 ----
c5, s5 = math.cos(Theta_5), math.sin(Theta_5)
c6, s6 = math.cos(Theta_6), math.sin(Theta_6)

x04x = (-s5 * ( T02*c1 + T12*s1)
        - c5 * ( s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1)))
x04y = (c5 * (T20*c6 - T21*s6) - T22*s5)

p13x = (d5 * (s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1))
        - d6 * (T02*c1 + T12*s1)
        + T03*c1 + T13*s1)

p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)




# ---- Theta 3 ----
c3 = (p13x**2 + p13y**2 - a2**2 - a3**2) / (2.0 * a2 * a3)
c3 = np.clip(c3, -1, 1)
Theta_3 = math.acos(c3)




# ---- Theta 2 ----
s3 = math.sin(Theta_3)
A23 = a2 + a3*c3
B23 = a3*s3
denom = A23**2 + B23**2

Theta_2 = math.atan2(
    (A23*p13y - B23*p13x) / denom,
    (A23*p13x + B23*p13y) / denom
)




# ---- Theta 4 ----
c23 = math.cos(Theta_2 + Theta_3)
s23 = math.sin(Theta_2 + Theta_3)

Theta_4 = math.atan2(
    c23*x04y - s23*x04x,
    x04x*c23 + x04y*s23
)

# ---- DH Function ----
def DH(theta, a, d, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

# ---- Joint Positions ----
def get_joint_positions(thetas):
    t1, t2, t3, t4, t5, t6 = thetas

    T_0_1 = DH(t1,  0,  d1,  math.pi/2)
    T_1_2 = DH(t2, a2,  0,   0)
    T_2_3 = DH(t3, a3,  0,   0)
    T_3_4 = DH(t4,  0,  d4,  math.pi/2)
    T_4_5 = DH(t5,  0,  d5, -math.pi/2)
    T_5_6 = DH(t6,  0,  d6,  0)

    T_0_2 = T_0_1 @ T_1_2
    T_0_3 = T_0_2 @ T_2_3
    T_0_4 = T_0_3 @ T_3_4
    T_0_5 = T_0_4 @ T_4_5
    T_0_6 = T_0_5 @ T_5_6

    return np.array([
        [0, 0, 0],
        T_0_1[:3, 3],
        T_0_2[:3, 3],
        T_0_3[:3, 3],
        T_0_4[:3, 3],
        T_0_5[:3, 3],
        T_0_6[:3, 3]
    ])

# ---- Print Results ----
print("\nJoint angles (degrees):")
print(f"Theta_1 = {math.degrees(Theta_1):.2f}")
print(f"Theta_2 = {math.degrees(Theta_2):.2f}")
print(f"Theta_3 = {math.degrees(Theta_3):.2f}")
print(f"Theta_4 = {math.degrees(Theta_4):.2f}")
print(f"Theta_5 = {math.degrees(Theta_5):.2f}")
print(f"Theta_6 = {math.degrees(Theta_6):.2f}")

# ---- Animation ----
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 1)
ax.set_title("UR5e Robot Arm IK Animation")

line, = ax.plot([], [], [], 'o-', linewidth=4)
target_dot, = ax.plot([X], [Y], [Z], 'ro', markersize=8)

start_angles = np.zeros(6)
end_angles = np.array([Theta_1, Theta_2, Theta_3, Theta_4, Theta_5, Theta_6])

steps = 100

def interpolate(i):
    alpha = i / steps
    return start_angles + alpha * (end_angles - start_angles)

def update(frame):
    angles = interpolate(frame)
    pts = get_joint_positions(angles)

    line.set_data(pts[:, 0], pts[:, 1])
    line.set_3d_properties(pts[:, 2])

    return line,

ani = FuncAnimation(fig, update, frames=steps, interval=50)

plt.show()