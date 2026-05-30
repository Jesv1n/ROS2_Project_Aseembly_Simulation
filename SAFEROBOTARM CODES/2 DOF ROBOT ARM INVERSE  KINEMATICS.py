# THIS PARTICULAR CODE GIVES THE INVERSE KINEMATICS AND FORWARD KINEMATICS MOVEMENT OF TWO LINKS OF A ROBOT ARM OF 2 DEGREE OF FREEDOM TO A CERTAIN POINT IN SPACE.INVERSE KINEMATICS TO FIND THETA_1 AND THETA_2 
# BOTH THE ANGLES OF BOTH THE LINK AND FORWARD KINEMATICS IS USED TO CALCULATE THE INTERMEDIATE END POINTS TO FIND THE POSITIONS IN BETWEEN THE BASE TO THE REQUIRED FINAL ENDPOINT
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Robot parameters ---
a2 = 5.9   # first link length
a4 = 6.0   # second link length
base_x, base_y = 0, 0

# --- Target point ---
X2, Y2 = 4, 10

# --- Compute distance to target ---
r = math.sqrt(X2**2 + Y2**2)
#max_reach = a2 + a4
#min_reach = abs(a2 - a4)

# --- Check if target is reachable ---

#if r > max_reach:
    #print("Target too far. Clamping to max reach.")
    #r = max_reach
#elif r < min_reach:
    #print("Target too close. Clamping to min reach.")
    #r = min_reach

# --- Inverse Kinematics with clamping ---
cos_phi1 = (a4**2 - r**2 - a2**2) / (-2 * r * a2)
#cos_phi1 = max(min(cos_phi1, 1), -1)   # clamp to [-1,1]
phi_1 = math.acos(cos_phi1)

phi_2 = math.atan2(Y2, X2)
theta1_final = phi_2 - phi_1
print(theta1_final)

cos_phi3 = (r**2 - a2**2 - a4**2) / (-2 * a2 * a4)
#cos_phi3 = max(min(cos_phi3, 1), -1)
phi_3 = math.acos(cos_phi3)
theta2_final = math.pi - phi_3
print(theta2_final)
# --- Animation setup ---
fig, ax = plt.subplots()
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_aspect('equal')
ax.grid(True)

line1, = ax.plot([], [], 'b-', linewidth=4)
line2, = ax.plot([], [], 'r-', linewidth=4)
target_dot, = ax.plot(X2, Y2, 'go', markersize=10)

# --- Interpolation for smooth motion ---
steps = 50
theta1_list = [i * theta1_final / steps for i in range(steps + 1)]
theta2_list = [i * theta2_final / steps for i in range(steps + 1)]

# --- Update function for animation ---
def update(frame):
    theta1 = theta1_list[frame]
    theta2 = theta2_list[frame]

    # Forward Kinematics
    x1 =  a2 * math.cos(theta1)
    y1 =  a2 * math.sin(theta1)
    x2 = x1 + a4 * math.cos(theta1 + theta2)
    y2 = y1 + a4 * math.sin(theta1 + theta2)

    # Update lines
    line1.set_data([base_x, x1], [base_y, y1])
    line2.set_data([x1, x2], [y1, y2])
    return line1, line2

# --- Run animation ---
ani = FuncAnimation(fig, update, frames=steps + 1, interval=50, blit=True)
plt.title("2-DOF Robot Arm Animation")
plt.show()