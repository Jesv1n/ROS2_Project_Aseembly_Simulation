# node creation --> client creation --> check current state --> the state we want
# --> for each position call the gripper --> switch it on
# --> go to home position --> then to the place position for scanner
# --> then vacuum sensor off --> then go to home position

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from builtin_interfaces.msg import Duration
import numpy as np
import math
import time
from collections import deque


class pickplace(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.gripper_client = self.create_client(
            SetIO,
            '/io_and_status_controller/set_io'
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        self.poses = [
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],   # home
                [-0.088, -2.109, -1.63, -1.011, 1.54, -0.39],  # pick-waypoint1
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],   # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint1
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 1.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.17, -1.42, -2.455, -0.89, 1.59, -0.26],  # place-waypoint 1
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242]   # home for 1
            ],
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133], # home
                [0.068, -1.98, -1.833, -0.9440, 1.56, -0.209], # pick-waypoint2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 2.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.24, -1.63, -2.23, -0.923, 1.53, -0.29], # place-waypoint 2
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home for 2
            ],
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [-0.213, -1.89, -2.03, -0.813, 1.522, -0.30], # pick-waypoint3
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint3
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 3.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.66, -1.217, -2.67, -0.83, 1.58, -0.36], # place-waypoint 3
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home for 3
            ],
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [-0.213, -1.89, -2.03, -0.813, 1.522, -0.30], # pick-waypoint4
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint3
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 3.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.66, -1.217, -2.67, -0.83, 1.58, -0.36], # place-waypoint 4
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home for 3
            ],
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [-0.213, -1.89, -2.03, -0.813, 1.522, -0.30], # pick-waypoint4
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint3
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 3.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.66, -1.217, -2.67, -0.83, 1.58, -0.36], # place-waypoint 4
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home for 3
            ],
            [
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133], # home
                [0.068, -1.98, -1.833, -0.9440, 1.56, -0.209], # pick-waypoint6
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # place-waypoint6
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [0.644, -1.703, -2.24, -0.80, 1.56, -0.25], # pick-waypoint 6.2
                [0.37, -0.92, -1.89, -1.80, 1.53, -0.133],  # home
                [3.66, -1.217, -2.67, -0.83, 1.58, -0.36], # place-waypoint 6
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home for 6
            ]
        ]

        self.placeoverflow = [
            [
                [2.85, -2.09, -1.712, -0.89, 1.62, 0.23],  # place the  overflow
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home
            ],
            [
                [2.77, -1.96, -1.94, -0.819, 1.58, 0.23],  # place the overflow
                [3.17, -1.06, -1.77, -1.79, 1.58, -0.242] # home
            ],
        ]

        self.declare_parameter("move_time", 6.0)
        self.declare_parameter("dt_waypoint", 0.02)
        self.declare_parameter("max_joint_velocity", 1.0)

        self.move_time = float(self.get_parameter("move_time").value)
        self.dt_waypoint = float(self.get_parameter("dt_waypoint").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)

        self.current_position = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.timer = self.create_timer(1.0, self.tick)
        self.step = 0
        self.count = 1
        self.trmp = 0
        self.count1 = 0
        self.count2 = 0
        self.full = 6
        self.full1 = 2
        self.ty = 0
        self.num = 0

        self.move_timer = None
        self.timer1 = None
        self.timer2 = None
        self.timer3 = None
        self.timer4 = None

        self.dq = deque()
        self.dq1 = deque()


    def set_on(self, name: str, fun, pin, level: float):
        self.req = SetIO.Request()
        self.req.fun = fun
        self.req.pin = pin
        self.req.state = level

        sending = self.gripper_client.call_async(self.req)
        sending.add_done_callback(lambda future: self.get_results(future, name))


    def get_results(self, future, name):
        res = future.result()
        if res.success:
            self.get_logger().info(f"worked {name}")
        else:
            self.get_logger().error(f"failed {name}")


    def joint_state_cb(self, msg: JointState):
        if self.current_position is None:
            self.get_logger().info("ready to receive the first joint state message as the current one is empty")
        pos_map = {name: i for i, name in enumerate(msg.name)}
        self.current_position = [msg.position[pos_map[name]] for name in self.joint_names]
         #  string[] name
        #  float64[] position
        #  float64[] velocity
        #  float64[] effort

 
    def tick(self):
        if self.current_position is None:
            return
        if self.move_timer is None:
            self.get_logger().info("Starting motion timer (every 8s). First move will start now")

            self.send_next_motion()
            self.set_on("Blower on", 3, 0, 0.7)
            self.set_on("Vaccum on", 1, 5, 1.0)
            self.set_on("Vaccum on", 1, 5, 0.0)

            self.move_timer = self.create_timer(8.0, self.send_next_motion)
            self.timer.cancel()



#------------------------------------------------------------------------------------------------------------------------------------------------------

    def send_next_motion(self):

        if self.count1 == self.full1:
            self.mode = 'Move'
            if self.dq1:
                self.kl = self.dq1.popleft()
            self.active_list = self.kl

        elif self.count == self.full:
            self.mode = 'Overflow'
            if self.dq:
                self.jk = self.dq.popleft()
            self.active_list = self.jk

        else:
            self.mode = 'Poses'
            self.active_list = self.poses[self.num]

        target = self.active_list[self.step]
        self.update(self.active_list, target)


    def update(self, pose_list, target):

        if self.current_position is None:
            self.get_logger().warning("No joint states available, skipping move.")
            return

        traj = self.build_smoothed_trajectory(
            self.current_position,
            target,
            desired_total_time=self.move_time,
            dt=self.dt_waypoint,
            max_vel=self.max_joint_velocity
        )

        self.pub.publish(traj)


        if self.trmp == 1:
            def vaccumpickon():
                self.set_on("valve pick on", 1, 7, 1.0)#----valvepickon
                def clear():
                    self.set_on("valve pick on", 1, 7, 0.0)#-----valvepickon
                    self.timer1.cancel()
                    self.timer2.cancel()
                self.timer2 = self.create_timer(0.2, clear)
            self.timer1 = self.create_timer(self.move_time, vaccumpickon)


        elif self.trmp == 3:
            def vaccumpickoff():
                self.set_on("valve pick off", 1, 6, 1.0) #-----valvepickoff
                def clear1():
                    self.set_on("valve pick off", 1, 6, 0.0)#----valvepickoff
                    self.timer3.cancel()
                    self.timer4.cancel()
                self.timer4 = self.create_timer(0.2, clear1)
            self.timer3 = self.create_timer(self.move_time, vaccumpickoff)
            self.trmp = -1

        if traj is None:
            self.get_logger().error("Failed to build trajectory.")
            return


        self.get_logger().info(f"Published smoothed trajectory to target: {[round(x,3) for x in target]} "
                               f"({len(traj.points)} points, total_time={traj.points[-1].time_from_start.sec}s)")

        print("length", len(pose_list))
        print("step", self.step)
        print("trmp", self.trmp)
        print("count", self.count)
        print("num", self.num)

        if self.step == len(pose_list) - 1:

            if self.mode == 'Overflow':
                self.count1 += 1
                self.step = 0
                self.num += 1
                if self.count1 == self.full1:
                    self.num = 0
                    self.trmp = 0

            elif self.mode == 'Move':
                self.count2 += 1
                if self.count2 == self.full1:
                    self.count1 -= 2
                    self.count -= 2
                    self.num = self.count
                    self.step = 0

            else:
                #last two position of poses andlast two position of placeoverflow
                #self.length2=len(self.placeoverflow[self.ty][-2])
                #self.n=[pose_list[:self.step-2]+[self.First_last,self.Second_last]]
                #self.first_six=self.poses[self.num][self.step-2:]

                self.length = len(self.placeoverflow[self.ty]) - 1
                self.First_last = self.placeoverflow[self.ty][self.length]
                self.Second_last = self.placeoverflow[self.ty][self.length - 1]
                self.last_two = self.poses[self.num][self.step : self.step-2 : -1]
                self.n=[self.last_two+[self.First_last,self.Second_last]]
                self.dq.append(self.n)

                # create a list of list to refill the taken from the ones for the overflow and to be used for move
                self.dq1.append(self.poses[self.num])
                self.step = 0
                self.count += 1
                self.num += 1

                if self.ty == self.full1:
                    self.ty = 0
                else:
                    self.ty = (self.ty + 1) % len(self.placeoverflow)

                self.trmp = -1

                if self.count == self.full:
                    self.num = 0
                    self.trmp = 0

        else:
            self.step = (self.step + 1) % len(pose_list)

        self.trmp += 1


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def build_smoothed_trajectory(self, start_positions, target_positions, desired_total_time=4.0, dt=0.2, max_vel=1.0):
        if len(start_positions) != len(self.joint_names) or len(target_positions) != len(self.joint_names):
            return None

        start = np.array(start_positions, dtype=float)
        target = np.array(target_positions, dtype=float)
        delta = target - start
        max_delta = np.max(np.abs(delta))

        min_time = 0.0
        if max_vel > 0:
            min_time = float(max_delta / max_vel)

        total_time = max(desired_total_time, min_time + 0.05)

           # compute number of intermediate points (including final point)
        n_points = max(2, int(math.ceil(total_time / dt)))
        time_step = total_time / float(max(1, n_points - 1))

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i in range(n_points):
            alpha = float(i) / float(max(1, n_points - 1))
            pos = (1.0 - alpha) * start + alpha * target
            pt = JointTrajectoryPoint()
            pt.positions = pos.tolist()

            # Set velocities to zero for safety and to satisfy UR requirement (final velocity must be zero).
            # With multiple points spaced in time, controller will interpolate smoothly based on times.

            pt.velocities = [0.0] * len(self.joint_names)
            pt.accelerations = [0.0] * len(self.joint_names)
            secs = int(math.floor(i * time_step))
            nanos = int(round((i * time_step - secs) * 1e9))
            pt.time_from_start = Duration(sec=secs, nanosec=nanos)
            traj.points.append(pt)

        return traj


# --------------------------- main ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = pickplace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_on("Vaccum off", 1, 4, 1.0)
        node.set_on("Vaccum off", 1, 4, 0.0)
        node.set_on("Blower off", 3, 0, 0.0)
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()