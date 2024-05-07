#   This node allows the user to input the desired injection coordinates in cartesian form (x,y,z) as
#   user_command_input topic and calculates the inverse kinematics for the 2 DoF injection system, 
#   returning la_1_pos ra_1_pos and la_2_pos - linear actuator 1 position (base linear actuator in mm), 
#   rotary actuator 1 position (rad), and linear actuator 2 position (final injector piezo actuator)
#   THe result is then sent to _____


#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from ethercat_motor_msgs.msg import MotorPositionMessage
from ethercat_motor_msgs.msg import EndEffectorTargetMessage
from ethercat_motor_msgs.msg import MotorCtrlMessage


# PARAMETERS

# Eye parameters
eyeDia = 25
eyeRad = eyeDia / 2
irisDia = 12
irisRad = irisDia / 2
pupilDia = 5
pupilRad = pupilDia / 2

# System settings
eyeRes = 500
platfEyeDist = 150              
gazeMargin = 8 * np.pi / 180
gazeAng = 20 * np.pi / 180
gazeAngMax = gazeAng + gazeMargin
gazeAngMin = gazeAng - gazeMargin
pupillaryDist = 63
eyeNoseDist = pupillaryDist / 2

# Injection parameters
minInjDistLimbus = 3
maxInjDistLimbus = 4
optimInjDistLimbus = minInjDistLimbus + (maxInjDistLimbus - minInjDistLimbus) / 2
optInjAngEye = optimInjDistLimbus / eyeRad + np.arcsin(irisRad / eyeRad)
optimInjRad = eyeRad * np.sin(optInjAngEye)
minInjAngEye = minInjDistLimbus / eyeRad + np.arcsin(irisRad / eyeRad)
maxInjAngEye = maxInjDistLimbus / eyeRad + np.arcsin(irisRad / eyeRad)
maxInjAngRoll = 80 * np.pi / 180
optInjAngHor = optInjAngEye - gazeAng
injectorAng = -25 * np.pi / 180         

# Actuator Hadrware Design Parameters
baseToPitchActDist = 0
pitchActToYActDist = 0
yActToInjDist = 73.5            
injLeng = 0

# Transformation Parameters

RB1 = np.eye(3)
R12 = lambda alph: np.array([[1, 0, 0], [0, np.cos(alph), -np.sin(alph)], [0, np.sin(alph), np.cos(alph)]])
R23 = np.array([[np.cos(injectorAng), -np.sin(injectorAng), 0], [np.sin(injectorAng), np.cos(injectorAng), 0], [0, 0, 1]])

r_B1_inB = np.array([baseToPitchActDist, 0, 0])
r_12_in1 = lambda y1: np.array([pitchActToYActDist, y1, 0])
r_23_in2 = np.array([0, yActToInjDist, 0])
r_3E_in3 = lambda y3: np.array([injLeng + y3, 0, 0])

# Transformation matrices and related calculations
H_B1 = np.block([[RB1, r_B1_inB[:, np.newaxis]], [0, 0, 0, 1]])
H_12 = lambda y1, alph: np.block([[R12(alph), r_12_in1(y1)[:, np.newaxis]], [0, 0, 0, 1]])
H_23 = np.block([[R23, r_23_in2[:, np.newaxis]], [0, 0, 0, 1]])

# Calculation of transformations from base to other reference frames
HB3 = lambda q: H_B1 @ H_12(q[0], q[1]) @ H_23
HB2 = lambda q: H_B1 @ H_12(q[0], q[1])

# Helper function to extract the upper 3x4 submatrix from a homogeneous transformation matrix
H_cut = lambda H: H[:3, :]

# Position vectors from one coordinate frame to another
r_B2_inB = lambda q: r_12_in1(q[0])
r_B3_inB = lambda q: H_cut(HB2(q)) @ np.append(r_23_in2, 1)
r_BE_inB = lambda q: H_cut(HB3(q)) @ np.append(r_3E_in3(q[2]), 1)

# Jacobian matrix calculation as a function of configuration vector q
jacob = lambda q: np.array([
    [0, 0, np.cos(injectorAng)],
    [1, -yActToInjDist * np.sin(q[1]) - q[2] * np.sin(q[1]) * np.sin(injectorAng), np.cos(q[1]) * np.sin(injectorAng)],
    [0, yActToInjDist * np.cos(q[1]) + q[2] * np.cos(q[1]) * np.sin(injectorAng), np.sin(q[1]) * np.sin(injectorAng)]
])

# Inverse Kinematic Function
"""
Perform inverse kinematics to find joint configuration that reaches a given end goal.

Args:
end_goal (np.array): The target end-effector position.

Returns:
tuple: A tuple containing the joint configuration (np.array) and a boolean indicating success (bool).
"""

def inv_kin(end_goal):

    bool_success = True
    q0 = np.array([0, 0, 0])  # Initial guess for the joint variables
    
    i_max = 500
    r_tolerance = 10e-6
    
    q = q0
    r = r_BE_inB(q)
    r_error = end_goal - r
    
    i = 1
    while np.max(np.abs(r_error)) > r_tolerance:
        # Calculate the pseudo-inverse of the Jacobian and update q
        q = q + np.linalg.pinv(jacob(q)) @ r_error
        
        r = r_BE_inB(q)
        r_error = end_goal - r
        i += 1
        
        if i >= i_max:
            print(f'No solution found within tolerance {r_tolerance} after {i} iterations!')
            bool_success = False
            break

    q_goal = q
    return q_goal, bool_success



# Test Point:
#138.2766
#32.0935
#4.2966

pub = None 
a1MaxStroke = 40
a2MaxRotation = 210 * np.pi / 180
a2MinRotation = -30 * np.pi / 180

def callback(data):

     
    endEffectorTargetPosition = np.array([0,0,0])
    motorPositions = np.array([0,0,0])
    endEffTargPos = np.array([0,0,0])
    motorMsg = MotorPositionMessage()
    validReach = False
    rospy.loginfo(endEffTargPos)

    endEffTargPos[0] = data.endEffTargPosX
    endEffTargPos[1] = data.endEffTargPosY
    endEffTargPos[2] = data.endEffTargPosZ

    rospy.loginfo("End Effector Target Position X(mm) %f", endEffTargPos[0])
    rospy.loginfo("End Effector Target Position Y(rad) %f", endEffTargPos[1])
    rospy.loginfo("End Effector Target Position Z(mm) %f", endEffTargPos[2])

    motorPositions, validReach = inv_kin(endEffTargPos)
    

    motorMsg.a1TargetPosition = round(motorPositions[0],3)
    motorMsg.a2TargetPosition = round(motorPositions[1],3)
    motorMsg.a3TargetPosition = round(motorPositions[2],3)

    rospy.loginfo("a1 Target Position X(mm) %f", motorMsg.a1TargetPosition)
    rospy.loginfo("a2 Effector Target Position Y(rad) %f", motorMsg.a2TargetPosition)
    rospy.loginfo("a3 Effector Target Position Z(mm) %f", motorMsg.a3TargetPosition)

        # Add another check to make sure hardware requirements are met.
    if abs(motorMsg.a1TargetPosition) <= a1MaxStroke and motorMsg.a2TargetPosition >= a2MinRotation and motorMsg.a2TargetPosition <= a2MaxRotation and validReach == True:
        publish_motor_command(motorMsg)
    else:
        rospy.loginfo("System cannot reach desired end-effector position.")

    
def inv_kin_node():

    rospy.loginfo("Started Inverse Kinematic Node!")

    # create the converter node
    rospy.init_node("inv_kin_node_1", anonymous=True)
    
    # create subscriber and publisher objects of node
    global pub
    pub = rospy.Publisher("motor_position_msg", MotorPositionMessage, queue_size=10)
    rospy.Subscriber("end_effector_target_msg", EndEffectorTargetMessage, callback)
    


def publish_motor_command(msg):

    pub.publish(msg)



if __name__ == '__main__':
    try:
        inv_kin_node()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





            

