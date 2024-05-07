#   This node allows the user to move the 2 dof kinematic system by inputing the desired
#   movement for the lin_act_1 and rot_act_1 (mm) and (deg) respectively.

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from ethercat_motor_msgs.msg import UserCtrlMessage

# Subscriber subscribes to 


def callback(data):
    user_msg_mmTargetPosition = data.mmTargetPosition
    user_msg_degTargetPosition = data.degTargetPosition

    rospy.loginfo("LA_1 target Pos (mm) %i", user_msg_mmTargetPosition)
    rospy.loginfo("RA_1 target Pos (deg) %i", user_msg_degTargetPosition)
    
def conversion_node():

    # create the converter node
    rospy.init_node("user_control_converter", anonymous=True)

    # 
    user_msg_mmTargetPosition = 0
    user_msg_degTargetPosition = 0

    # create subscriber and publisher objects of node
    rospy.Subscriber("user_command_input", UserCtrlMessage, callback)
    pub = rospy.Publisher('motor_command_output', Int32, queue_size=10)


def publish_motor_command():

    pub.publish(user_msg_mmTargetPosition)
    pub.publish(user_msg_degTargetPosition)
    

if __name__ == '__main__':
    try:
        conversion_node()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
