#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
import math
import matplotlib.pyplot as plt

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.nsecs
        t_start = time_now/1e9

        
        l1 = 0.267
        l2 = 0.293
        l3 = 0.0525
        l4 = 0.3512
        l5 = 0.1232
        th1 = 0.2225 #(rad) (=12.75deg)
        th2 = 0.6646 #(rad) (=38.08deg)
        
        dy = 0
        y_acc = 0
        s =0
        y0 = 0
        yf = 0.2

        

        dt = 0
        t = 0 
        y_graph = []
        times = []
       
        joint1 = []
        joint2 = []
        joint3 = []
        joint4 = []
        joint5 = []
        joint6 = []
        joint7 = []



        periods = 0
        errory = []
        errorx = []
        errorz = []
        right_obstacle = 1
        left_obstacle = 0

        min1 = []
        min2 = []
        first = True

        while not rospy.is_shutdown():

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A01 = self.kinematics.tf_A01(self.joint_angpos)

            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)

            # pseudoinverse jacobian
            pinvJ = pinv(J)

            ################### FIRST TASK ##########################
            edef =self.kinematics.tf_A07(self.joint_angpos) 
            x_edef = edef[0,3]
            y_edef = edef[1,3]
            z_edef = edef[2,3]
            x_edef = math.trunc(x_edef*10000) / 10000
            y_edef = math.trunc(y_edef*10000) / 10000
            z_edef = math.trunc(z_edef*10000) / 10000

            linex=0.6043
            linez=0.1508
            y_graph.append(y_edef)
           	
            
            if (y_edef >= 0.2) & (right_obstacle==1):
                y0 = 0.2
                yf = -0.2
                t = 0
                periods = periods + tf
                right_obstacle =0
                left_obstacle = 1

            if (y_edef<= -0.2) & (left_obstacle==1):
                y0=-0.2
                yf=0.2
                t=0
                periods = periods + tf
                left_obstacle = 0
                right_obstacle =1

            

            
            tf = 4
            t = t + dt
            

            times.append(t+periods)

            
            a0 = y0
            a1 = 0
            a2 = 0
            a3 = -(10/(tf **3))*(y0-yf)
            a4 = (15/(tf  **4))*(y0-yf)
            a5 = -(6/(tf  **5))*(y0-yf)
            s =a5*t **5 + a4*t **4 + a3*t **3 + a2*t**2 + a1*t + a0
            dy = 5*a5*t **4 + 4*a4*t **3 + 3*a3*t**2 + 2*a2*t  + a1

            
            errory.append(y_edef-s)
            errorz.append(z_edef-linez)
            errorx.append(x_edef-linex)

            
            pdot = np.matrix([[20*(linex -x_edef) ],\
                        [dy ],\
                        [20*(linez -z_edef) ]])
            
            task1 = np.dot(pinvJ,pdot)
            
            ################### SECOND TASK #######################
            
            #find the position of the centers of the two obstacles
            yobst1=self.model_states.pose[1].position.y  #green
            yobst2=self.model_states.pose[2].position.y  #red
            #print('OI THESEIS TWN EMPODIWN EINAI:::::::::::', yobst1, yobst2)
            
            #find the joint's positions
            self.A07=self.kinematics.tf_A07(self.joint_angpos)
            self.A06=self.kinematics.tf_A06(self.joint_angpos)
            self.A05=self.kinematics.tf_A05(self.joint_angpos)
            self.A04=self.kinematics.tf_A04(self.joint_angpos)
            self.A03=self.kinematics.tf_A03(self.joint_angpos)

            #compute each distance of the joints of the robot from the two obstacles
            
            dobst1=np.sqrt((self.A07[0,3]-0.3)**2+(self.A07[1,3]-yobst1)**2)
            dobst2=np.sqrt((self.A07[0,3]-0.3)**2+(self.A07[1,3]-yobst2)**2)
            d6obst1=np.sqrt((self.A06[0,3]-0.3)**2+(self.A06[1,3]-yobst1)**2)
            d6obst2=np.sqrt((self.A06[0,3]-0.3)**2+(self.A06[1,3]-yobst2)**2)
            d5obst1=np.sqrt((self.A05[0,3]-0.3)**2+(self.A05[1,3]-yobst1)**2)
            d5obst2=np.sqrt((self.A05[0,3]-0.3)**2+(self.A05[1,3]-yobst2)**2)
            d4obst1=np.sqrt((self.A04[0,3]-0.3)**2+(self.A04[1,3]-yobst1)**2)
            d4obst2=np.sqrt((self.A04[0,3]-0.3)**2+(self.A04[1,3]-yobst2)**2)
            d3obst1=np.sqrt((self.A03[0,3]-0.3)**2+(self.A03[1,3]-yobst1)**2)
            d3obst2=np.sqrt((self.A03[0,3]-0.3)**2+(self.A03[1,3]-yobst2)**2)
            d45x = (self.A05[0,3] + self.A04[0,3])/2
            d45y = (self.A05[1,3] + self.A04[1,3])/2 
            d45 = np.sqrt((d45x -0.3)**2+(d45y -yobst1-0.03)**2)
            
            #find the minimum distance
            minimum1=min(dobst1,d6obst1,d5obst1,d4obst1,d3obst1,d45)
            minimum2=min(dobst2,d6obst2,d5obst2,d4obst2,d3obst2)

            
            
            #find the derivative of the criterion function

           
            q1 = self.joint_states.position[0]
            q2 = self.joint_states.position[1]
            q3 = self.joint_states.position[2]
            q4 = self.joint_states.position[3]
            q5 = self.joint_states.position[4]
            q6 = self.joint_states.position[5]
            q7 = self.joint_states.position[6]

            Jy7, Jy6, Jy5, Jy4, Jy3, Jx7, Jx6, Jx5, Jx4, Jx3 = self.kinematics.compute_Jacobian_of_closest_point(q1,q2,q3,q4,q5,q6,q7,th1,th2)
            
            if minimum1==dobst1 :
                grad1=2*np.dot(self.A07[0,3]-0.3,Jx7) + 2*np.dot(self.A07[1,3]-yobst1,Jy7)
            elif minimum1==d6obst1 :
                grad1=2*np.dot(self.A06[0,3]-0.3,Jx6) + 2*np.dot(self.A06[1,3]-yobst1,Jy6)
            elif minimum1==d5obst1 :
                grad1=2*np.dot(self.A05[0,3]-0.3,Jx5) + 2*np.dot(self.A05[1,3]-yobst1,Jy5)
            elif minimum1==d4obst1 :
                grad1=2*np.dot(self.A04[0,3]-0.3,Jx4) + 2*np.dot(self.A04[1,3]-yobst1,Jy4)
            elif minimum1==d3obst1 :   
                grad1=2*np.dot(self.A03[0,3]-0.3,Jx3) + 2*np.dot(self.A03[1,3]-yobst1,Jy3)
            elif minimum1==d45:
                #print('d45')
                grad1= 2*np.dot(d45x-0.3,Jx3) + 2*np.dot(d45y-yobst1-0.06,Jy3)
                #print('dextra')
            
            if minimum2==dobst2 :
                grad2=2*np.dot(self.A07[0,3]-0.3,Jx7) + 2*np.dot(self.A07[1,3]-yobst2,Jy7)
            elif minimum2==d6obst2 :
                grad2=2*np.dot(self.A06[0,3]-0.3,Jx6) + 2*np.dot(self.A06[1,3]-yobst2,Jy6)          
            elif minimum2==d5obst2 :                
                grad2=2*np.dot(self.A05[0,3]-0.3,Jx5) + 2*np.dot(self.A05[1,3]-yobst2,Jy5)
            elif minimum2==d4obst2 :
                grad2=2*np.dot(self.A04[0,3]-0.3,Jx4) + 2*np.dot(self.A04[1,3]-yobst2,Jy4)
            else :
                grad2=2*np.dot(self.A03[0,3]-0.3,Jx3) + 2*np.dot(self.A03[1,3]-yobst2,Jy3)
        
            
            
            k2=10
            kc=15
            Kc=np.dot(np.eye(7),kc)

            #let us see which obstacle is closer
            if minimum1<minimum2:
                task2 = k2*np.dot((np.eye(7)-np.dot(pinvJ,J)),np.dot(Kc,grad1.transpose())) 
            
            else:
                task2 = k2*np.dot((np.eye(7)-np.dot(pinvJ,J)),np.dot(Kc,grad2.transpose())) 

            
            if (minimum1<0.16) | (minimum2< 0.15):
                #print('############# minimum1 IS: #####################', minimum1)
                task=task1 +task2
            else:
                task=task1 


            for i in range (0,7):
                self.joint_angvel[i] = task[i,0] 

            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.nsecs
            
            dt = (time_now - time_prev)/1e9
            if dt<0:
            	dt = 1 +dt 

            # Integration
            for i in range (0,7):
                self.joint_angpos[i] = self.joint_angpos[i] + dt * self.joint_angvel[i]
            joint1.append(self.joint_angpos[0])
            joint2.append(self.joint_angpos[1]) 
            joint3.append(self.joint_angpos[2]) 
            joint4.append(self.joint_angpos[3])
            joint5.append(self.joint_angpos[4]) 
            joint6.append(self.joint_angpos[5]) 
            joint7.append(self.joint_angpos[6]) 

            min1.append(minimum1)
            min2.append(minimum2)


            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            '''
            ############################ PRINT THE DIAGRAMS ########################
            j =1

            print("##################round(t) is:###############", t)
             
            if (t/4>1 and first):
                
                print('IM IN HEREEEEEEEEEEEEEEEEEEEEEEEE')
                plt.figure()
                plt.plot(times,y_graph)
                plt.title( "End-effector's position at y-axis")
                plt.xlabel("t")
                plt.show()

                plt.figure()
                plt.plot(times,errory)
                plt.title( "Position error at y-axis")
                plt.xlabel("t")
                plt.show()

                plt.figure()
                plt.plot(times,errorx)
                plt.title( "Position error at x-axis")
                plt.xlabel("t")
                plt.show()

                plt.figure()
                plt.plot(times,errorz)
                plt.title( "Position error at z-axis")
                plt.xlabel("t")
                plt.show()
                
                plt.figure()
                plt.plot(times,min1)
                plt.title( "Robot's distance from green obstacle")
                plt.xlabel("t")
                plt.show()

                plt.figure()
                plt.plot(times,min2)
                plt.title( "Robot's distance from red obstacle")
                plt.xlabel("t")
                plt.show()
                
                for i in [joint1,joint2,joint3,joint4,joint5,joint6,joint7]:
                    plt.figure()
                    plt.plot(times,i)
                    plt.title( "joints"+str(j)+" angular positions")
                    plt.xlabel("t")
                    plt.ylabel("joint"+str(j))
                    j+=1
                    plt.show()
                
                #first = False
            
            #prnt('##################### joint1##################', joint1)
            #print('##################### joint2##################', joint2)
            #print('##################### joint3##################', joint3)
            #print('##################### joint4##################', joint4)
            #print('##################### joint5##################', joint5)
            #print('##################### joint6##################', joint6)
            #print('##################### joint7##################', joint7)
            #print('##################### y_graph##################', y_graph)
            #print('##################### times##################', times)
            #print('##################### errory##################', errory)
            #print('##################### errorx##################', errorx)
            #print('##################### errorz##################', errorz)
            #print('##################### min1 ##################', min1)
            #print('##################### min2 ##################', min2)
            '''


            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
