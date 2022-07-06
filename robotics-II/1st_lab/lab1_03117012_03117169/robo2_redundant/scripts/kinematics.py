#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):
    	q1 = r_joints_array[0]
    	q2 = r_joints_array[1]
    	q3 = r_joints_array[2]
    	q4 = r_joints_array[3]
    	q5 = r_joints_array[4]
    	q6 = r_joints_array[5]
    	q7 = r_joints_array[6]

    	l1 = self.l1
    	l2 = self.l2
    	l3 = self.l3
    	l4 = self.l4
    	l5 = self.l5

    	theta1 = self.theta1 
    	theta2 = self.theta2 

    	J_11 = l5*np.cos(theta2)*(1.0*np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) - 1.0*l4*np.cos(theta1)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l2*np.sin(q1)*np.sin(q2) - l5*np.sin(theta2)*(np.sin(q6)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2)) + np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3)))) - 1.0*l4*np.sin(theta1)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l3*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1))
    	J_12 = l2*np.cos(q1)*np.cos(q2) + l5*np.cos(theta2)*(np.cos(q1)*np.sin(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) - np.cos(q1)*np.cos(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) - l5*np.sin(theta2)*(np.cos(q1)*np.cos(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) + np.cos(q1)*np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) - 1.0*l3*np.cos(q1)*np.cos(q3)*np.sin(q2) - 1.0*l4*np.cos(q1)*np.cos(theta1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) + 1.0*l4*np.cos(q1)*np.sin(theta1)*(np.cos(q2)*np.sin(q4) - np.cos(q3)*np.cos(q4)*np.sin(q2))
    	J_13 = l5*np.cos(theta2)*(np.sin(q6)*(np.sin(q5)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q4)*np.cos(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - 1.0*np.cos(q6)*np.sin(q4)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - l5*np.sin(theta2)*(np.cos(q6)*(np.sin(q5)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q4)*np.cos(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) + np.sin(q4)*np.sin(q6)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - 1.0*l3*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - 1.0*l4*np.cos(q4)*np.sin(theta1)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - 1.0*l4*np.cos(theta1)*np.sin(q4)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))
    	J_14 = 1.0*l4*np.sin(theta1)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2)) - l5*np.sin(theta2)*(np.sin(q6)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.cos(q5)*np.cos(q6)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))) - l5*np.cos(theta2)*(np.cos(q6)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)) + 1.0*np.cos(q5)*np.sin(q6)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))) - 1.0*l4*np.cos(theta1)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4))
    	J_15 = -1.0*l5*np.sin(q6 - 1.0*theta2)*(np.cos(q3)*np.cos(q5)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.cos(q5)*np.sin(q3) - np.cos(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q4)*np.sin(q1)*np.sin(q3)*np.sin(q5) - np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q5))
    	J_16 = l5*np.sin(theta2)*(np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2))) + l5*np.cos(theta2)*(1.0*np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) + np.sin(q6)*(1.0*np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2)))
    	J_17 = 0

    	J_21 = l5*np.cos(theta2)*(1.0*np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2))) - l5*np.sin(theta2)*(np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) + np.sin(q6)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2))) - 1.0*l3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*l2*np.cos(q1)*np.sin(q2) - 1.0*l4*np.sin(theta1)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.cos(theta1)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))
    	J_22 = l2*np.cos(q2)*np.sin(q1) + l5*np.cos(theta2)*(np.sin(q1)*np.sin(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) - np.cos(q6)*np.sin(q1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) - l5*np.sin(theta2)*(np.cos(q6)*np.sin(q1)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) + np.sin(q1)*np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) - 1.0*l3*np.cos(q3)*np.sin(q1)*np.sin(q2) - 1.0*l4*np.cos(theta1)*np.sin(q1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) + 1.0*l4*np.sin(q1)*np.sin(theta1)*(np.cos(q2)*np.sin(q4) - np.cos(q3)*np.cos(q4)*np.sin(q2))
    	J_23 = 1.0*l3*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3)) + l5*np.sin(theta2)*(np.cos(q6)*(np.sin(q5)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.cos(q4)*np.cos(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) + 1.0*np.sin(q4)*np.sin(q6)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) - l5*np.cos(theta2)*(1.0*np.sin(q6)*(np.sin(q5)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.cos(q4)*np.cos(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) - 1.0*np.cos(q6)*np.sin(q4)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) + 1.0*l4*np.cos(q4)*np.sin(theta1)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3)) + 1.0*l4*np.cos(theta1)*np.sin(q4)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))
    	J_24 = l5*np.cos(theta2)*(np.cos(q6)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4)) + 1.0*np.cos(q5)*np.sin(q6)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2))) - 1.0*l4*np.sin(theta1)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2)) + l5*np.sin(theta2)*(np.sin(q6)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.cos(q5)*np.cos(q6)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2))) + 1.0*l4*np.cos(theta1)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4))
    	J_25 = 1.0*l5*np.sin(q6 - 1.0*theta2)*(np.cos(q1)*np.cos(q3)*np.cos(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3) + np.cos(q1)*np.cos(q4)*np.sin(q3)*np.sin(q5) + np.sin(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q5))
    	J_26 = - l5*np.sin(theta2)*(np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) - l5*np.cos(theta2)*(1.0*np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) + np.sin(q6)*(1.0*np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2)))
    	J_27 = 0

    	J_31 = 0
    	J_32 = 1.0*l4*np.cos(theta1)*(np.cos(q4)*np.sin(q2) - np.cos(q2)*np.cos(q3)*np.sin(q4)) - 1.0*l2*np.sin(q2) - 1.0*l4*np.sin(theta1)*(np.sin(q2)*np.sin(q4) + 1.0*np.cos(q2)*np.cos(q3)*np.cos(q4)) + l5*np.sin(theta2)*(np.sin(q6)*(np.cos(q4)*np.sin(q2) - np.cos(q2)*np.cos(q3)*np.sin(q4)) - np.cos(q6)*(np.cos(q5)*(np.sin(q2)*np.sin(q4) + 1.0*np.cos(q2)*np.cos(q3)*np.cos(q4)) + 1.0*np.cos(q2)*np.sin(q3)*np.sin(q5))) - 1.0*l3*np.cos(q2)*np.cos(q3) + l5*np.cos(theta2)*(np.cos(q6)*(np.cos(q4)*np.sin(q2) - np.cos(q2)*np.cos(q3)*np.sin(q4)) + 1.0*np.sin(q6)*(np.cos(q5)*(np.sin(q2)*np.sin(q4) + 1.0*np.cos(q2)*np.cos(q3)*np.cos(q4)) + 1.0*np.cos(q2)*np.sin(q3)*np.sin(q5)))
    	J_33 = l3*np.sin(q2)*np.sin(q3) + 1.0*l5*np.sin(q2)*np.sin(theta2)*(np.sin(q3)*np.sin(q4)*np.sin(q6) - np.cos(q3)*np.cos(q6)*np.sin(q5) + np.cos(q4)*np.cos(q5)*np.cos(q6)*np.sin(q3)) + l5*np.cos(theta2)*np.sin(q2)*(np.cos(q6)*np.sin(q3)*np.sin(q4) + np.cos(q3)*np.sin(q5)*np.sin(q6) - 1.0*np.cos(q4)*np.cos(q5)*np.sin(q3)*np.sin(q6)) + l4*np.cos(q4)*np.sin(q2)*np.sin(q3)*np.sin(theta1) + l4*np.cos(theta1)*np.sin(q2)*np.sin(q3)*np.sin(q4)
    	J_34 = 1.0*l4*np.cos(theta1)*(np.cos(q2)*np.sin(q4) - np.cos(q3)*np.cos(q4)*np.sin(q2)) + 1.0*l4*np.sin(theta1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) + l5*np.sin(theta2)*(np.sin(q6)*(np.cos(q2)*np.sin(q4) - np.cos(q3)*np.cos(q4)*np.sin(q2)) + np.cos(q5)*np.cos(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) + l5*np.cos(theta2)*(np.cos(q6)*(np.cos(q2)*np.sin(q4) - np.cos(q3)*np.cos(q4)*np.sin(q2)) - 1.0*np.cos(q5)*np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)))
    	J_35 = 1.0*l5*np.sin(q6 - 1.0*theta2)*(np.cos(q5)*np.sin(q2)*np.sin(q3) + np.cos(q2)*np.sin(q4)*np.sin(q5) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q2)*np.sin(q5))
    	J_36 = l5*np.cos(theta2)*(np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) - 1.0*np.cos(q6)*(np.cos(q5)*(np.cos(q2)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q2)) - 1.0*np.sin(q2)*np.sin(q3)*np.sin(q5))) - l5*np.sin(theta2)*(np.cos(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) + np.sin(q6)*(np.cos(q5)*(np.cos(q2)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q2)) - 1.0*np.sin(q2)*np.sin(q3)*np.sin(q5)))
    	J_37 = 0

    	J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
    	return J
        
        
    def compute_Jacobian_of_closest_point(self,q1,q2,q3,q4,q5,q6,q7,th1,th2):

        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        Jx70 =l5*np.cos(th2)*(1.0*np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) - 1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l2*np.sin(q1)*np.sin(q2) - l5*np.sin(th2)*(np.sin(q6)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2)) + np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q1)*np.cos(q3) - np.cos(q2)*np.sin(q1)*np.sin(q3)))) - 1.0*l4*np.sin(th1)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l3*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1))
        Jx71 =l2*np.cos(q1)*np.cos(q2) - l5*np.cos(th2)*(1.0*np.cos(q1)*np.cos(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) - np.cos(q1)*np.sin(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2))) - 1.0*l5*np.sin(th2)*(np.cos(q1)*np.cos(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) + np.cos(q1)*np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) + 1.0*l4*np.cos(q1)*np.sin(th1)*(np.cos(q2)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q2)) - 1.0*l3*np.cos(q1)*np.cos(q3)*np.sin(q2) - 1.0*l4*np.cos(q1)*np.cos(th1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))
        Jx72 =l5*np.cos(th2)*(np.sin(q6)*(np.sin(q5)*(np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q4)*np.cos(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - 1.0*np.cos(q6)*np.sin(q4)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - 1.0*l5*np.sin(th2)*(np.cos(q6)*(np.sin(q5)*(np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q4)*np.cos(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) + np.sin(q4)*np.sin(q6)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - 1.0*l3*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - 1.0*l4*np.cos(q4)*np.sin(th1)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - 1.0*l4*np.cos(th1)*np.sin(q4)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))
        Jx73 =1.0*l4*np.sin(th1)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2)) - 1.0*l5*np.sin(th2)*(np.sin(q6)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)) - 1.0*np.cos(q5)*np.cos(q6)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))) - 1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l5*np.cos(th2)*(np.cos(q6)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)) + 1.0*np.cos(q5)*np.sin(q6)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2)))
        Jx74 =-1.0*l5*np.sin(q6 - 1.0*th2)*(np.cos(q3)*np.cos(q5)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.cos(q5)*np.sin(q3) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q4)*np.sin(q1)*np.sin(q3)*np.sin(q5) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q5))
        Jx75 =l5*np.cos(th2)*(np.sin(q6)*(1.0*np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2)) - 1.0*np.cos(q6)*(1.0*np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - np.cos(q5)*(np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4)))) - l5*np.sin(th2)*(1.0*np.cos(q6)*(1.0*np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2)) + np.sin(q6)*(1.0*np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3)) - np.cos(q5)*(np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4))))
        Jx76 =0
        Jy70 =l5*np.cos(th2)*(1.0*np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) - np.cos(q6)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2))) - l5*np.sin(th2)*(np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - np.sin(q5)*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))) + np.sin(q6)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*np.cos(q1)*np.cos(q4)*np.sin(q2))) - 1.0*l3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + 1.0*l2*np.cos(q1)*np.sin(q2) - 1.0*l4*np.sin(th1)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))
        Jy71 =l2*np.cos(q2)*np.sin(q1) - l5*np.cos(th2)*(1.0*np.cos(q6)*np.sin(q1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4)) - np.sin(q1)*np.sin(q6)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2))) - 1.0*l5*np.sin(th2)*(np.cos(q6)*np.sin(q1)*(np.sin(q2)*np.sin(q3)*np.sin(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q4) + np.cos(q3)*np.cos(q4)*np.cos(q5)*np.sin(q2)) + np.sin(q1)*np.sin(q6)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))) + 1.0*l4*np.sin(q1)*np.sin(th1)*(np.cos(q2)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q2)) - 1.0*l3*np.cos(q3)*np.sin(q1)*np.sin(q2) - 1.0*l4*np.cos(th1)*np.sin(q1)*(np.cos(q2)*np.cos(q4) + np.cos(q3)*np.sin(q2)*np.sin(q4))
        Jy72 =1.0*l3*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3)) + l5*np.sin(th2)*(np.cos(q6)*(np.sin(q5)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.cos(q4)*np.cos(q5)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))) + 1.0*np.sin(q4)*np.sin(q6)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))) - 1.0*l5*np.cos(th2)*(1.0*np.sin(q6)*(np.sin(q5)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.cos(q4)*np.cos(q5)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))) - 1.0*np.cos(q6)*np.sin(q4)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))) + 1.0*l4*np.cos(q4)*np.sin(th1)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3)) + 1.0*l4*np.cos(th1)*np.sin(q4)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))
        Jy73 =l5*np.cos(th2)*(np.cos(q6)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4)) + 1.0*np.cos(q5)*np.sin(q6)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) + l5*np.sin(th2)*(np.sin(q6)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + 1.0*np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*np.cos(q5)*np.cos(q6)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) + 1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.sin(th1)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))
        Jy74 =1.0*l5*np.sin(q6 - 1.0*th2)*(np.cos(q1)*np.cos(q3)*np.cos(q5) - 1.0*np.cos(q2)*np.cos(q5)*np.sin(q1)*np.sin(q3) + np.cos(q1)*np.cos(q4)*np.sin(q3)*np.sin(q5) + np.sin(q1)*np.sin(q2)*np.sin(q4)*np.sin(q5) + np.cos(q2)*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q5))
        Jy75 =- 1.0*l5*np.sin(th2)*(np.sin(q6)*(np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*np.sin(q5)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))) - 1.0*np.cos(q6)*(1.0*np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))) - 1.0*l5*np.cos(th2)*(np.sin(q6)*(1.0*np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2)) + 1.0*np.cos(q6)*(np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*np.sin(q5)*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))))
        Jy76 =0

        Jy60 =1.0*l2*np.cos(q1)*np.sin(q2) - 1.0*l3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*l4*np.sin(th1)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - 			1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))
        Jy61 =1.0*l4*np.sin(th1)*(np.cos(q2)*np.sin(q1)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l4*np.cos(th1)*(np.cos(q2)*np.cos(q4)*np.sin(q1) + 1.0*np.cos(q3)*np.sin(q1)*np.sin(q2)*np.sin(q4)) + 1.0*l2*np.cos(q2)*np.sin(q1) - 1.0*l3*np.cos(q3)*np.sin(q1)*np.sin(q2)
        Jy62 =(l3 + l4*np.sin(q4 + th1))*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))
        Jy63 =1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.sin(th1)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))
        Jy64 =0
        Jy65 =0
        Jy66 =0
        Jx60 =- 1.0*l3*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l2*np.sin(q1)*np.sin(q2) - 1.0*l4*np.sin(th1)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4))
        Jx61 =1.0*l4*np.sin(th1)*(np.cos(q1)*np.cos(q2)*np.sin(q4) - 1.0*np.cos(q1)*np.cos(q3)*np.cos(q4)*np.sin(q2)) + 1.0*l2*np.cos(q1)*np.cos(q2) - 1.0*l4*np.cos(th1)*(np.cos(q1)*np.cos(q2)*np.cos(q4) + 1.0*np.cos(q1)*np.cos(q3)*np.sin(q2)*np.sin(q4)) - 1.0*l3*np.cos(q1)*np.cos(q3)*np.sin(q2)
        Jx62 =-1.0*(l3 + l4*np.sin(q4 + th1))*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))
        Jx63 =1.0*l4*np.sin(th1)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2)) - 1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4))
        Jx64 =0
        Jx65 =0
        Jx66 =0

        Jy50 =1.0*l2*np.cos(q1)*np.sin(q2) - 1.0*l3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - l4*np.sin(th1)*(np.cos(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) - np.cos(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2))
        Jy51 =l4*np.sin(th1)*(np.cos(q2)*np.sin(q1)*np.sin(q4) - 1.0*np.cos(q3)*np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l4*np.cos(th1)*(np.cos(q2)*np.cos(q4)*np.sin(q1) + 1.0*np.cos(q3)*np.sin(q1)*np.sin(q2)*np.sin(q4)) + 1.0*l2*np.cos(q2)*np.sin(q1) - 1.0*l3*np.cos(q3)*np.sin(q1)*np.sin(q2)
        Jy52 =(l3 + l4*np.sin(q4 + th1))*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))
        Jy53 =1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4)) - 1.0*l4*np.sin(th1)*(np.sin(q4)*(np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*np.cos(q4)*np.sin(q1)*np.sin(q2))
        Jy54 =0
        Jy55 =0
        Jy56 =0
        Jx50 =- 1.0*l3*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*l4*np.cos(th1)*(1.0*np.sin(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - np.cos(q4)*np.sin(q1)*np.sin(q2)) - 1.0*l2*np.sin(q1)*np.sin(q2) - l4*np.sin(th1)*(np.cos(q4)*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q1)*np.sin(q2)*np.sin(q4))
        Jx51 =l4*np.sin(th1)*(np.cos(q1)*np.cos(q2)*np.sin(q4) - 1.0*np.cos(q1)*np.cos(q3)*np.cos(q4)*np.sin(q2)) + 1.0*l2*np.cos(q1)*np.cos(q2) - 1.0*l4*np.cos(th1)*(np.cos(q1)*np.cos(q2)*np.cos(q4) + 1.0*np.cos(q1)*np.cos(q3)*np.sin(q2)*np.sin(q4)) - 1.0*l3*np.cos(q1)*np.cos(q3)*np.sin(q2)
        Jx52 =-1.0*(l3 + l4*np.sin(q4 + th1))*(np.cos(q3)*np.sin(q1) + np.cos(q1)*np.cos(q2)*np.sin(q3))
        Jx53 =l4*np.sin(th1)*(np.sin(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) + np.cos(q1)*np.cos(q4)*np.sin(q2)) - 1.0*l4*np.cos(th1)*(1.0*np.cos(q4)*(1.0*np.sin(q1)*np.sin(q3) - 1.0*np.cos(q1)*np.cos(q2)*np.cos(q3)) - 1.0*np.cos(q1)*np.sin(q2)*np.sin(q4))
        Jx54 =0
        Jx55 =0
        Jx56 =0

        Jy40 =1.0*l2*np.cos(q1)*np.sin(q2) - l3*(np.sin(q1)*np.sin(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3))
        Jy41 =np.sin(q1)*(l2*np.cos(q2) - 1.0*l3*np.cos(q3)*np.sin(q2))
        Jy42 =l3*(np.cos(q1)*np.cos(q3) - 1.0*np.cos(q2)*np.sin(q1)*np.sin(q3))
        Jy43 =0
        Jy44 =0
        Jy45 =0
        Jy46 =0
        Jx40 =- l3*(1.0*np.cos(q1)*np.sin(q3) + np.cos(q2)*np.cos(q3)*np.sin(q1)) - 1.0*l2*np.sin(q1)*np.sin(q2)
        Jx41 =np.cos(q1)*(l2*np.cos(q2) - 1.0*l3*np.cos(q3)*np.sin(q2))
        Jx42 =- 1.0*l3*np.cos(q3)*np.sin(q1) - 1.0*l3*np.cos(q1)*np.cos(q2)*np.sin(q3)
        Jx43 =0
        Jx44 =0
        Jx45 =0
        Jx46 =0

        Jy30 =l2*np.cos(q1)*np.sin(q2)
        Jy31 =l2*np.cos(q2)*np.sin(q1)
        Jy32 =0
        Jy33 =0
        Jy34 =0
        Jy35 =0
        Jy36 =0
        Jx30 =-l2*np.sin(q1)*np.sin(q2)
        Jx31 =l2*np.cos(q1)*np.cos(q2)
        Jx32 =0
        Jx33 =0
        Jx34 =0
        Jx35 =0
        Jx36 =0


        Jy7= np.matrix([[Jy70 , Jy71 , Jy72 , Jy73, Jy74, Jy75, Jy76]])
        Jy6= np.matrix([[Jy60 , Jy61 , Jy62 , Jy63, Jy64, Jy65, Jy66]])
        Jy5= np.matrix([[Jy50 , Jy51 , Jy52 , Jy53, Jy54, Jy55, Jy56]])
        Jy4= np.matrix([[Jy40 , Jy41 , Jy42 , Jy43, Jy44, Jy45, Jy46]])
        Jy3= np.matrix([[Jy30 , Jy31 , Jy32 , Jy33, Jy34, Jy35, Jy36]])
        Jx7= np.matrix([[Jx70 , Jx71 , Jx72 , Jx73, Jx74, Jx75, Jx76]])
        Jx6= np.matrix([[Jx60 , Jx61 , Jx62 , Jx63, Jx64, Jx65, Jx66]])
        Jx5= np.matrix([[Jx50 , Jx51 , Jx52 , Jx53, Jx54, Jx55, Jx56]])
        Jx4= np.matrix([[Jx40 , Jx41 , Jx42 , Jx43, Jx44, Jx45, Jx46]])
        Jx3= np.matrix([[Jx30 , Jx31 , Jx32 , Jx33, Jx34, Jx35, Jx36]])
            
        return Jy7, Jy6, Jy5, Jy4, Jy3, Jx7, Jx6, Jx5, Jx4, Jx3

    def tf_A01(self, r_joints_array):
    	q1 = r_joints_array[0]
    	l1 = self.l1
    	tf = np.matrix([[np.cos(q1) , -np.sin(q1) , 0 , 0],\
                        [np.sin(q1) , np.cos(q1) , 0 , 0],\
                        [0 , 0 , 1 , l1],\
                        [0 , 0 , 0 , 1]])
    	return tf

    def tf_A02(self, r_joints_array):
    	q2 = r_joints_array[1]
    	tf_A12 = np.matrix([[np.cos(q2) , -np.sin(q2) , 0 , 0],\
                            [0 , 0 , 1 , 0],\
                            [-np.sin(q2) , -np.cos(q2) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
    	return tf

    def tf_A03(self, r_joints_array):
    	q3 = r_joints_array[2]
    	l2 = self.l2
    	tf_A23 = np.matrix([[np.cos(q3) , -np.sin(q3) , 0 , 0],\
                            [0 , 0 , -1 , -l2],\
                            [np.sin(q3) , np.cos(q3) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
    	return tf

    def tf_A04(self, r_joints_array):
    	q4 = r_joints_array[3]
    	l3 = self.l3
    	tf_A34 = np.matrix([[np.cos(q4) , -np.sin(q4) , 0 , l3],\
                            [0 , 0 , -1 , 0],\
                            [np.sin(q4) , np.cos(q4) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
    	return tf

    def tf_A05(self, r_joints_array):
    	q5 = r_joints_array[4]
    	l4 = self.l4
    	theta1 = self.theta1
    	tf_A45 = np.matrix([[np.cos(q5) , -np.sin(q5) , 0 , l4*np.sin(theta1)],\
                            [0 , 0 , -1 , -l4*np.cos(theta1)],\
                            [np.sin(q5) , np.cos(q5) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
    	return tf

    def tf_A06(self, r_joints_array):
    	q6 = r_joints_array[5]
    	tf_A56 = np.matrix([[np.cos(q6) , -np.sin(q6) , 0 , 0],\
                            [0 , 0 , -1 , 0],\
                            [np.sin(q6) , np.cos(q6) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
    	return tf

    def tf_A07(self, r_joints_array):
    	q7 = r_joints_array[6]
    	l5 = self.l5
    	theta2 = self.theta2 
    	tf_A67 = np.matrix([[np.cos(q7) , -np.sin(q7) , 0 , l5*np.sin(theta2)],\
                            [0 , 0 , 1 , l5*np.cos(theta2)],\
                            [-np.sin(q7) , -np.cos(q7) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
    	tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
    	return tf
