import numpy as np

# THIS IS TO  UNDERSTAND HOW THE ROTATION MATRIX LOOKS LIKE WHEN THERE IS NOT ROTATIONS IN THE SERVO
servo_0_angle= 12
servo_1_angle= 63

servo_0_angle=np.deg2rad(servo_0_angle)
servo_1_angle=np.deg2rad(servo_1_angle)

#NAKE THE FIRST ROTATION MATRIX

rot_matrix_0_1=np.array([[np.cos(servo_0_angle),-np.sin(servo_0_angle),0],
                         [np.sin(servo_0_angle),np.cos(servo_0_angle),0],
                         [0,0,1]])

rot_matrix_1_2=np.array([[np.cos(servo_1_angle),-np.sin(servo_1_angle),0],
                         [np.sin(servo_1_angle),np.cos(servo_1_angle),0],
                         [0,0,1]])

rot_matrix_0_2=np.matmul(rot_matrix_0_1,rot_matrix_1_2)

print(rot_matrix_0_2)