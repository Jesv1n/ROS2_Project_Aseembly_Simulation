import numpy as np

# THIS IS TO  UNDERSTAND HOW THE ROTATION MATRIX LOOKS LIKE WHEN THERE IS NOT ROTATIONS IN THE SERVO
servo_0_angle= -77.10
servo_1_angle= 5.69
servo_2_angle= 236.47
servo_3_angle= -12.03
servo_4_angle= 28.63
servo_5_angle= 8.06


servo_0_angle=np.deg2rad(servo_0_angle)
servo_1_angle=np.deg2rad(servo_1_angle)
servo_2_angle=np.deg2rad(servo_2_angle)
servo_3_angle=np.deg2rad(servo_3_angle)
servo_4_angle=np.deg2rad(servo_4_angle)
servo_5_angle=np.deg2rad(servo_5_angle)

#NAKE THE FIRST ROTATION MATRIX

rot_matrix_0_1=np.array([[np.cos(servo_0_angle),0,np.sin(servo_0_angle)],
                         [np.sin(servo_0_angle),0,-np.cos(servo_0_angle)],
                         [0,1,0]])


rot_matrix_1_2=np.array([[np.cos(servo_1_angle),-np.sin(servo_1_angle),0],
                         [np.sin(servo_1_angle),np.cos(servo_1_angle),0],
                         [0,0,1]])


rot_matrix_2_3=np.array([[np.cos(servo_2_angle),0,np.sin(servo_2_angle)],
                         [np.sin(servo_2_angle),0,-np.cos(servo_2_angle)],
                         [0,1,0]])


rot_matrix_3_4=np.array([[np.cos(servo_3_angle),0,-np.sin(servo_3_angle)],
                         [np.sin(servo_3_angle),0,np.cos(servo_3_angle)],
                         [0,-1,0]])



rot_matrix_4_5=np.array([[np.cos(servo_4_angle),0,np.sin(servo_4_angle)],
                         [np.sin(servo_4_angle),0,-np.cos(servo_4_angle)],
                         [0,1,0]])

rot_matrix_5_6=np.array([[np.cos(servo_5_angle),-np.sin(servo_5_angle),0],
                         [np.sin(servo_5_angle),np.cos(servo_5_angle),0],
                         [0,0,1]])



rot_matrix_0_6=rot_matrix_0_1@rot_matrix_1_2@rot_matrix_2_3@rot_matrix_3_4@rot_matrix_4_5@rot_matrix_5_6
rot_matrix_0_3 = rot_matrix_0_1 @ rot_matrix_1_2 @ rot_matrix_2_3

rot_matrix_0_3_T = rot_matrix_0_3.T

rot_matrix_3_6 = rot_matrix_0_3_T @ rot_matrix_0_6
print(rot_matrix_0_6)

#print(rot_matrix_0_3)

#print(rot_matrix_0_3_T)

#print(rot_matrix_3_6)