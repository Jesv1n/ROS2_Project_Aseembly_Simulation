import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class inverse(Node):

    def __init__(self):
        super().__init__('controller')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Read actual joint positions before sending
        self._start_angles = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        while self._start_angles is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for controller to connect
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.send_traj()

    def _js_cb(self, msg):
        if self._start_angles is not None:
            return
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self._start_angles = np.array(
            [msg.position[msg.name.index(n)] for n in joint_names]
        )

    def send_traj(self):

        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # ---- Target Pose ----
        pose = [3.6670, 0.5791, 0.2101, 0.0, np.pi, 1.50]
        X, Y, Z, roll, pitch, yaw = pose

        # ---- UR5e DH Parameters ----
        d1 =  0.1625
        a2 = -0.425
        a3 = -0.3922
        d4 =  0.1333
        d5 =  0.0997
        d6 =  0.1996

        # ---- Rotation Matrix ----
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw),  math.cos(yaw), 0],
                       [0, 0, 1]])

        Ry = np.array([[ math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])

        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll),  math.cos(roll)]])

        R = Rz @ Ry @ Rx
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [X, Y, Z]

        T00, T01, T02, T03 = T[0]
        T10, T11, T12, T13 = T[1]
        T20, T21, T22, T23 = T[2]

        # ---- Theta 1 
        A = d6 * T12 - T13
        B = d6 * T02 - T03
        R_sq = A * A + B * B

        arccos_val = math.acos(np.clip(d4 / math.sqrt(R_sq), -1, 1))
        arctan_val = math.atan2(-B, A)

        def norm(a):
            return math.atan2(math.sin(a), math.cos(a))

        T1_a = norm( arccos_val + arctan_val)
        T1_b = norm(-arccos_val + arctan_val)
        Theta_1 = T1_a if abs(T1_a - self._start_angles[0]) \
                       <= abs(T1_b - self._start_angles[0]) else T1_b

        # ---- Theta 5 — pick branch closest to current position ----
        c1, s1 = math.cos(Theta_1), math.sin(Theta_1)
        numer = T03 * s1 - T13 * c1 - d4
        acos5 = math.acos(np.clip(numer / d6, -1, 1))
        Theta_5 = acos5 if abs(acos5 - self._start_angles[4]) \
                        <= abs(-acos5 - self._start_angles[4]) else -acos5

        # ---- Theta 6 ----
        Theta_6 = math.atan2(
            -(T01 * s1 - T11 * c1),
             (T00 * s1 - T10 * c1)
        )

        # ---- Theta 2, 3, 4 ----
        c5, s5 = math.cos(Theta_5), math.sin(Theta_5)
        c6, s6 = math.cos(Theta_6), math.sin(Theta_6)

        x04x = (-s5 * (T02 * c1 + T12 * s1)
                - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1)))
        x04y = (c5 * (T20 * c6 - T21 * s6) - T22 * s5)

        p13x = (d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1))
                - d6 * (T02 * c1 + T12 * s1)
                + T03 * c1 + T13 * s1)
        p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6)

        c3 = (p13x ** 2 + p13y ** 2 - a2 ** 2 - a3 ** 2) / (2.0 * a2 * a3)
        c3 = np.clip(c3, -1, 1)
        Theta_3 = math.acos(c3)

        s3 = math.sin(Theta_3)
        A23 = a2 + a3 * c3
        B23 = a3 * s3
        denom = A23 ** 2 + B23 ** 2

        Theta_2 = math.atan2(
            (A23 * p13y - B23 * p13x) / denom,
            (A23 * p13x + B23 * p13y) / denom
        )

        c23 = math.cos(Theta_2 + Theta_3)
        s23 = math.sin(Theta_2 + Theta_3)

        Theta_4 = math.atan2(
            c23 * x04y - s23 * x04x,
            x04x * c23 + x04y * s23
        )

        # ---- Print ----
        print("\nJoint angles (degrees):")
        print(f"Theta_1 = {math.degrees(Theta_1):.2f}")
        print(f"Theta_2 = {math.degrees(Theta_2):.2f}")
        print(f"Theta_3 = {math.degrees(Theta_3):.2f}")
        print(f"Theta_4 = {math.degrees(Theta_4):.2f}")
        print(f"Theta_5 = {math.degrees(Theta_5):.2f}")
        print(f"Theta_6 = {math.degrees(Theta_6):.2f}")

        start_angles = self._start_angles
        end_angles = np.array([Theta_1, Theta_2, Theta_3, Theta_4, Theta_5, Theta_6])

        steps = 100
        total_time = 1.0

        for i in range(steps):
            alpha = i / (steps - 1)
            t = alpha * total_time

            # Smoothstep: zero velocity at start and end
            s     =  3 * alpha**2 - 2 * alpha**3
            ds_dt = (6 * alpha   - 6 * alpha**2) / total_time

            point = JointTrajectoryPoint()
            point.positions  = (start_angles + s * (end_angles - start_angles)).tolist()
            point.velocities = (ds_dt * (end_angles - start_angles)).tolist()
            point.time_from_start.sec     = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(point)

        msg.header.stamp.sec     = 0
        msg.header.stamp.nanosec = 0

        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory sent!')


def main(args=None):
    rclpy.init(args=args)
    node = inverse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()