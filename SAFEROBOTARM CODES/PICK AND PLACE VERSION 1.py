import rclpy
from trajectory_msgs
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from sensor_msgs.msg import JOintStaes
from builtin_interfaces.msg import Duration
import numpy as np

class SMoothedPickPlace(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.pub=self.create_publsiher( JointTrajectory,'/scaled_joint_trajectory_controller/joint_trajectory',10)

    self.joint_names=[
        'shoulder_pan_joint',
        'shoulder_lift_joint',                     #this seflf  means it belongs to this node
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    self.poses=[
        [3.720,-1.187,-1.795,-1.437,1.509,0.227], #home
        [3.208,-1.686,-1.784,-1.286,1.509,0.227], #pick
        [4.163,-1.831,-1.637,-1.449,1.652,0.227], #plcae
    ]

    self.declare_parameter("move_time",6.0)
    self.declare_parameter("dt_waypoint",0.02)            #this is in ROS environement  as paarmeter dictionary not python variable it lives in ROS  pythoin  variable lives in the program
    self.declare_parameter("max_joint_velocity",1.0)



    self.move_time=float(self.get_parameter("move_time").value)
    self.dt_waypoint=float(self.get_parameter("dt_waypoint").value)
    self.max_joint_velocity=float(self.get_parameter("max_joint_velocity").value)


    self.current_joint_position=None
    self.joinr_state_sub=self.create_subscriber(JointState,'/joint_states', self.joint_state_cb ,10)  #--whenever the function is called it is passed as joint_state_cb(self,msg)
    # it can be msg.name or msg.position...
    #--here msg.name is string that is the message coming from the joint_position as 'shouldr_pan_joint,.....


    self.step=0
    self.timer=self.create_timer(1.0,self.tick)  #---call the function every 1 sec
    self.move_timer=None



def joint_state_cd(self,msg:JointState):  #----extract joint states and storte them in cureent_joint_position in correct order
      if self.current_joint_position is None:
           self.get_logger().info("RECEVIED THE FIRTST JOINT STATE MESSAGE")
      pose_map={name:i for i,name in enumerate(msg.name)}      #---we are creating a dictionary to save joint name with the index 
                                                                #--msg.name comes from the ROS system from JOint State
                                                                # --instead of adding counters and going through the numbers we can actually give numbers for enumerating

      try:
           self.current_joint_positions = [msg.position[pos_map[name]]for name in self.joint_names]
                                                                # ---fist thye for loop workss taking the name of each joint_names
                                                                #--- the names we have provided in the beginning  and pos_map[name]
                                                                #-- works next giving the index that is the pos_map is a dictionary
                                                                # so when asked a key it gives the valeue
                                                                # now it has msg.position[0]---> in each positions like 1.20...
                                                                # save in the current_joint_position
      except KeyError:
           self.get_logger().warning("joint state message  doesnot contain all expected  joint names")



#--function fro timer
def tick(self):
     if self.current_joint_position is None:
          return
     if self.move_timer is None:
          self.get_logger().info("starting motyion timer every 8s ")

          self.send_next_motion()     #-----------first motion start immmediaTELY so this is the beginning motion
          self.move_timer= self.create_timer(8.0,self.send_next_motion) #---------next motion in 8 sec after begining

          self.timer.cancel() #---once the timer tick is called in 1 sec it runs the send_next_motion
                              #---so we actually doesnot required the first ticvk timner



def send_next_motion(self):
     
     target = self.poses[self.step] #---first pose i.e 0  i.e [3.720,-1.187....]
     self.step=(self.step+1)%len(self.poses)  #---,ore like a count but limited to 3 and reset to 0 when 3


     if self.current_joint_position is NOne:   #---- if no jopint staes are avialble
          self.logger().warning(no joint state available,skipping move)
          return
     
     traj = self.build_smoothed_jrajectory(self.current_joint_positions,target,
                                           desired_totoal_time=self.move_time,
                                           dt=self.dt_waypoint,
                                           max_vel=self.max_joint_velocity)  #---this is a call back function called


     if traj is None:
          self.get_logger().error("failed to build trajectory")
          return

     self.pub.publish(traj)     
     self.get_logger().info(f".........................") #---just logging info nothing else


     def build_smoothed_trajectory(self,start_position,target_positions,desired_total_time=4.0,dt=0.2,max_vel=1.0):
          if len(start_position) != len(self.joint_names) or len(target_positions) != len(self.joint_names):
                                                            #---we are defining the function with parameters these parameters
                                                            #-- and next we chaecked whether all the joint names are there by checking their lengthg
            return None
          start= np.array(start_position,dtype=float)   #--we convert to numpy arrauy because  list doesnot have soem operation and its hard
          target=np.array(target_positions,dtype=float)
          
          delta = target - start  #---shows how much the joints need to move

          max_delta= np.max(np.abs(delta))  #---inorder to find the largest joint movement ignoring the sign
                                            #--like think about the max_vel is 1.0if joint moves 0.9 rad the min time taken by it is 0.9/1=0.9
          min_time=0.0

          if max_vel>0:
               min_time= float(max_delta/max_vel) #-----like time = distance/velocity

          total_time=max(desired_total_time,min_time+0.05) #---we wont allow total time to be samll

          n_point= max(2,int(math.ceil(total_time/dt))) #----no fractional ewaypoint so use ceil.we need minimum 2 waypoint

          time_step= total_time/float(max(1,n_point-1)) #---time between each waypoint  i.e if total_time if 6.0 and n_point 300
                                                        #---we get 0.002sec that is one waypoints to another waypoint is 0.02sec


          traj =JointTrajectory()

          traj.joint_names=self.joint_names #--- A jointtrajectory message contains list of joint names and list of jointrajectory point


          for i in range(n_point):
               alpha =float(i)/float(max(1,n_point-1))  #---progress point where it should be at  position calculation

               pos = (1.0-alpha)*start+alpha*target #--combines the shrinking part of the start position with growing part of the
                                                    #target position

              
               pt=JointTrajectoryPoint()
               pt.positions =pos.tolist() #----so jointtrajectory as whole contrain  
                                            #---JointTrajectory:
                                            #      joint_names :[shoulde.....]
                                            #           points:
                                            #              -position:[]
                                            #               velocities:[]
               pt.velocity=[0.0]*len(self.joint_names)
               pt.accelerations=[0.0]*len(self.joint_names)

                                  #----set both to zeros fro smooth movement as the the robot controller will interpolate smoothly between points using positions and timing.

               secs= int(math.floor((i*time_step))) #---calculate secomnds floor round
               nanos=int(round((i*time_step-sec)*1e9))#--calculate anano sec for presin


               pt.time_from_start=Duration(sec=secs,nanosec=nanos) 

               traj.points.append(pt)                  
          return traj

         
          

def main(args=None):
    rclpy.init(args=args)
    node = SmoothedPickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

















