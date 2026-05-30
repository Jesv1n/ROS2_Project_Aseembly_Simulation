#node creation-->client creatiojn-->check current state-->the state we want--for each position call the gripper -->switch it on -->
#go to home position-->then to the place position for scanner-->then vaccum sensor off -->then go to home position

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

        self.pub = self.create_publisher(JointTrajectory,'/scaled_joint_trajectory_controller/joint_trajectory',10)

        self.gripper_client = self.create_client(SetIO,'/io_and_status_controller/set_io')

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
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint1
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint1
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 1.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 1
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 1
             ],
             [
                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 2.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 2
             ],

             [

                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint3
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint3
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 3.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 3
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 3

             ],

            [
                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint4
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint4
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 4.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 4
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 4

            ],

            [

                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint5
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint5
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 5.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 5
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 5
            ],
            
            [

                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint6
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint6
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 6.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 6
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 6

            ],

        ]


        self.placeoverflow=[
             
             [
                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 2.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 2
             ],
             [

                [-0.002, -2.152, -1.557, -1.068, 1.596, 0.039],  # pick-waypoint2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # pick-waypoint 2.2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home
                [0.938, -1.637, -2.282, -0.881, 1.572, 0.950],  # place-waypoint 2
                [0.345, -1.470, -1.597, -1.657, 1.526, 0.039],  # home for 2

             ],
            
        ]


        self.declare_parameter("move_time",6.0)
        self.declare_parameter("dt_waypoint",0.02)                           # we gave in ros environment  converrt it to our code
        self.declare_parameter("max_joint_velocity",1.0)

        self.move_time = float(self.get_parameter("move_time").value)
        self.dt_waypoint = float(self.get_parameter("dt_waypoint").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)

        self.current_position = None
        self.joint_state_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_cb,10)

        self.timer = self.create_timer(1.0,self.tick)
        self.step = 0
        self.count = 1
        self.trmp=0  
        self.count1=0 #--counter for placenew
        self.count2=0
        self.full=6#--this is the factor ewhich control vaccum
        self.full1=2
        self.ty=0

        self.num=0 # for calling list of list
        self.move_timer = None
        self.timer1 = None   
        self.timer2 = None  
        self.timer3 = None   
        self.timer4 = None   

        self.dq = deque()




    def set_on(self,name:str,fun,pin,level:float):
        self.req = SetIO.Request()
        self.req.fun = fun
        self.req.pin = pin
        self.req.state = level

        sending = self.gripper_client.call_async(self.req)
        sending.add_done_callback(lambda future:self.get_results(future,name))

    def get_results(self,future, name):
        res = future.result()
        if res.success:
            self.get_logger().info(f"worked {name}")
        else:
            self.get_logger().error(f"failed {name}")

    def joint_state_cb(self,msg:JointState):
        if self.current_position is None:
            self.get_logger().info("ready to recive the first joint state messgae as the current one is empty")    #---------Header header

        pos_map = {name:i for i ,name in enumerate(msg.name)}   
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
            self.set_on("Blower on",3,0,0.7)
            self.set_on("Vaccum on",1,5,1.0)
            self.set_on("Vaccum on",1,5,0.0)

            self.move_timer = self.create_timer(8.0,self.send_next_motion) 

            self.timer.cancel()

    #------------------------------------------------------------------------------------------------------------------------------------------------------

    def send_next_motion(self):

        if self.count1 == self.full1:
            self.mode = 'Move'
            if self.dq:
                self.kl = self.dq.pop()

            self.active_list = self.kl
             

        elif self.count == self.full:
            self.mode='Overflow'
            self.active_list= self.placeoverflow[self.num]

        else:
            self.mode='Poses'
            self.active_list = self.poses[self.num]

        target= self.active_list[self.step]
        self.update(self.active_list,target)
        
    def update(self,pose_list,target):
               

                if self.current_position is None:
                        self.get_logger().warning("No joint states available, skipping move.")
                        return
                    
                traj = self.build_smoothed_trajectory(self.current_position, target,
                                                        desired_total_time=self.move_time,
                                                        dt=self.dt_waypoint,
                                                        max_vel=self.max_joint_velocity)
                    
                self.pub.publish(traj)

                    
                if self.trmp == 1:

                        def vaccumpickon():
                            self.set_on("valve pick on",1,7,1.0)#-- valvepick on
                            def clear():
                                self.set_on("valve pick on",1,7,0.0)
                                self.timer1.cancel()
                                self.timer2.cancel()
                            self.timer2=self.create_timer(0.2,clear)

                        self.timer1=self.create_timer(self.move_time,vaccumpickon)

                elif self.trmp == 3:

                        def vaccumpickoff():
                            self.set_on("valve pick off",1,6,1.0)#-- valvepick off
                            def clear1():
                                self.set_on("valve pick off",1,6,0.0)#-- valvepick off
                                self.timer3.cancel()
                                self.timer4.cancel()
                            self.timer4=self.create_timer(0.2,clear1)  
                        self.timer3=self.create_timer(self.move_time,vaccumpickoff)
                        self.trmp=-1


                if traj is None:
                        self.get_logger().error("Failed to build trajectory.")
                        return

                self.get_logger().info(f"Published smoothed trajectory to target: {[round(x,3) for x in target]} "
                                        f"({len(traj.points)} points, total_time={traj.points[-1].time_from_start.sec}s)")
                    
                if self.step==len(pose_list)-1:
                        if self.mode == 'Overflow':
                             self.count1=self.count1+1
                             self.step=0
                             self.num=self.num+1
                             if self.num==self.count1:
                                  self.num=0
                            
                        
                        elif self.mode == 'Move':
                            self.count2=self.count2+1
                            if self.count2==self.full1:
                                self.count1=self.count1-2
                                self.count=self.count-2
                                self.num=self.count
                                self.step=0


                            
                        
                        else:
                             self.length=len(self.placeoverflow[self.ty])-1
                             self.First_last=self.placeoverflow[self.ty][self.length]
                             self.Second_last=self.placeoverflow[self.ty][self.length-1]
                             #self.length2=len(self.placeoverflow[self.ty][-2])
                             self.n=[pose_list[self.step],pose_list[self.step-1],self.First_last,self.Second_last]
                             self.dq.append(self.n)
                             self.step=0
                             self.count=self.count+1
                             self.num=self.num+1
                             self.ty=(self.ty+1)% len(self.placeoverflow)
                             if self.num==self.count-1:
                                  self.num=0
                             
                              
                             

                else:
                        self.step = (self.step + 1) % len(pose_list)
                       

                self.trmp=self.trmp+1
                
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def build_smoothed_trajectory(self, start_positions, target_positions, desired_total_time=4.0, dt=0.2, max_vel=1.0):
        if len(start_positions) != len(self.joint_names) or len(target_positions) != len(self.joint_names):
            return None

        start = np.array(start_positions, dtype=float)
        target = np.array(target_positions, dtype=float)
        delta = target - start
        max_delta = np.max(np.abs(delta))

        # compute minimal time required by max joint velocity
        min_time = 0.0
        if max_vel > 0:
            min_time = float(max_delta / max_vel)

        # pick total time at least the desired one but not less than min_time
        total_time = max(desired_total_time, min_time + 0.05)  

        # compute number of intermediate points (including final point)
        n_points = max(2, int(math.ceil(total_time / dt)))
        time_step = total_time / float(max(1, n_points - 1))

        # build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i in range(n_points):
            alpha = float(i) / float(max(1, n_points - 1))  # 0..1
            pos = (1.0 - alpha) * start + alpha * target

            pt = JointTrajectoryPoint()
            pt.positions = pos.tolist()

            # Set velocities to zero for safety and to satisfy UR requirement (final velocity must be zero).
            # With multiple points spaced in time, controller will interpolate smoothly based on times.
            pt.velocities = [0.0] * len(self.joint_names)
            pt.accelerations = [0.0] * len(self.joint_names)

            # assign increasing time_from_start
            secs = int(math.floor((i * time_step)))
            nanos = int(round((i * time_step - secs) * 1e9))
            pt.time_from_start = Duration(sec=secs, nanosec=nanos)

            traj.points.append(pt)

        return traj

def main(args=None):
    rclpy.init(args=args)
    node = pickplace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_on("Vaccum off",1,4,1.0)
        node.set_on("Vaccum off",1,4,0.0)
        node.set_on("Blower off",3,0,0.0)
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
