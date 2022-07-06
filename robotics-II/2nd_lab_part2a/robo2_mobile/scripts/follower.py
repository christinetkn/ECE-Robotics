#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
import matplotlib.pyplot as plt

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])


#######################################################################################
######################################### TEAM 20 #####################################
#############################   KOUNOUDIS DIMITRIS  03117169  ##########################
############################## TSAKANIKA XRISTINA    03117012 #########################
### Parameters:  angle = mod(11,pi) = 1.57522 --> 90.25 degrees, 11 is odd so counterclockwise #####
######################################################################################

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)
        
    

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(10)
        tmp_rate.sleep()
        print("The system is executing the algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
         
        start_time=rostime_now

        t=0
        error=0
        stage1a=True
        stage2=True
        seconds=0
        stop=False
        error_list=[]
        x_linear=[]
        x_linear_acceleration=[]
        z_angular=[]
        z_angular_acceleration=[]
        time=[]

        while not rospy.is_shutdown():
            
            # wz: angular velocity at z axis
            # vx: linear velocity at x axis
            # ax: linear acceleration at x axis
            # ang_accel_z: angular acceleration at z axis
            # seconds: each "frame" of the simulation
            # stage1a: wallfollowing stage
            #stage2: the stage when we have to stop the robot, after completing the circle
            
            seconds += 1
            
            #print("exery time seconds is : ", seconds)
            sonar_front = self.sonar_F.range
            sonar_frontleft=self.sonar_FL.range
            sonar_frontright=self.sonar_FR.range
            sonar_left=self.sonar_L.range
            sonar_right=self.sonar_R.range            
            
            #this is the first stage where the robot doesn't have walls close to it. We have set a distance threshold to 0.25.
            #So it moves straight ahead to the direction of the angle until the distance bocomes less than 0.25
            # Firslty it accelerates (until t = 0.5sec) and after it moves with constant speed
            #The robot will be in that stage only for once and never again
            
            if min(sonar_front,sonar_frontleft,sonar_frontright) >= 0.25 :
                
                previous_error = error
                angle = atan ((sonar_frontright*cos(pi/4) - sonar_right) / (sonar_frontright*sin(pi/4)));
                error = - cos(angle)*sonar_right + 0.25
            
                
                if (t <= 0.5):
                        #compute the acceleration polyonyms:
                        vx =40*t**3-120*t**4+96*t**5
                        ax =120*t**2-360*t**3+480*t**4
                        wz = 0
                        ang_accel_z = 0
                        t = t + 0.1; 	
                else:
                        #when  t>=0.5 the robot moves with constant speed
                        vx = 0.5
                        ax = 0 
                        wz = 0
                        ang_accel_z = 0
                self.velocity.linear.x = vx
                self.velocity.angular.z = -wz 
            
            elif(seconds>=9900):
                        
                        #when we have one total circle it's time to reduce the veloicty and finally stop the vehicle
                        if (stage2):
                            t = 0.1;
                            vx = 0.5;
                            ax =0
                            stage2=False
                        if (t <= 0.5):
                            vx = 0.5-(40*t**3-120*t**4+96*t**5)
                            ax = -(120*t**2-360*t**3+480*t**4)
                            t = t + 0.1
                        else:
                            vx = 0
                            
                            """
                            ###############Plot our graphs#####################
                            plt.plot(time,error_list)
                            plt.xlabel('Time (sec)')
                            plt.ylabel('Error (m)')
                            plt.title('Distance Error From the Walls')
                            plt.show(block=True)
                            
                            plt.plot(time,x_linear)
                            plt.xlabel('Time (sec)')
                            plt.ylabel('Linear Velocity (m/s)')
                            plt.title('Linear Velocity of  Robot ')
                            plt.show(block=True)
                            
                            plt.plot(time,x_linear_acceleration)
                            plt.xlabel('Time (sec)')
                            plt.ylabel('Linear Acceleration (m/s^2)')
                            plt.title('Linear Acceleration of  Robot ')
                            plt.show(block=True)
                            
                            plt.plot(time,z_angular)
                            plt.xlabel('Time (sec)')
                            plt.ylabel('Angular Velocity (rad/s)')
                            plt.title('Angular Velocity of Robot ')
                            plt.show(block=True)
                            
                            plt.plot(time,z_angular_acceleration)
                            plt.xlabel('Time (sec)')
                            plt.ylabel('Angular Acceleration (rad/s)')
                            plt.title('Angular Acceleration of Robot ')
                            plt.show(block=True)
                            """
                        self.velocity.linear.x = vx
                        self.velocity.angular.z = 0 
            
            # In this stage we decide:
            # if there are two walls in front of the robot, we rotate it and make ti parallel to the second wall
            # if there is no second wall, the robot continues wall-following
            else:
                previous_error = error
                #angle describes the angular deviation between thw anted angle and the one that the robot has.
                #angle: how many radiants is the robot rotated in repsect of the parallel wall
                #the sonar_frontright and sonar_right lasers form an angle fo 45 degrees --> pi/4
                angle = atan ((sonar_frontright*cos(pi/4) - sonar_right) / (sonar_frontright*sin(pi/4)))
                
                #we add to the error the distance from the wall
                error = - cos(angle)*sonar_right + 0.25
                
                if (sonar_front <= 0.25 or sonar_frontright <=0.25+0.018 or sonar_frontleft <=0.25+0.018):	
                   
                   #if we are in the wall-following stage, move with the constant speed of 0.5m/sec
                   if (stage1a):
                      t = 0.1
                      vx = 0.5
                      wz = 0
                      ang_accel_z = 0
                      stage1a=False
                   #if we have second wall close to us we wall-follow reducing our speed
                   if (t <= 0.5):
                      vx = 0.5 -(40*t**3-120*t**4+96*t**5)
                      ax = -(120*t**2-360*t**3+480*t**4)
                      wz = -(40*t**3-120*t**4+96*t**5)
                      ang_accel_z = -(120*t**2-360*t**3+480*t**4)
                      t = t + 0.1;
                   #this is the stage before every wall-folllowing. the robot   is stopped in a parallel position in front of the second wall
                   #in this stage we rotate the robot in order to becoe parallel to the second wall
                   else:
                      vx = 0;
                      wz = -0.5;
                   self.velocity.linear.x = vx
                   self.velocity.angular.z = wz
                # finish wall following be setting linear vlocity equal to zero   
                else:
                   if (stage1a): 
                      t = 0.1;
                      vx = 0;
                      stage1a=False
                   #first we try to reach the wanted speed of 0.5m/sec.
                   if (t <= 0.5):
                      vx = 40*t**3-120*t**4+96*t**5
                      t = t + 0.1
                   #we continue wall following with v = 0.5m/sec
                   else:
                      vx = 0.5
                   self.velocity.angular.z = -(12.5 * (error) + 11.0 * (error - previous_error))
                   self.velocity.linear.x = vx
             
             
            x_linear.append(self.velocity.linear.x)
            x_linear_acceleration.append(ax)
            z_angular.append(self.velocity.angular.z)
            z_angular_acceleration.append(ang_accel_z)
            error_list.append(abs(error))
            time.append(seconds*0.1/2)
            
            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
