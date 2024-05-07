#   This node converts the desired actuator positions returned by the inv-kin node given the user-specified     
#   end-effector position into motor control commands

#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from ethercat_motor_msgs.msg import MotorPositionMessage
from ethercat_motor_msgs.msg import MotorCtrlMessage

# Hardware Parameters
    #   a1 (IGUS Drylin SHT-12 Linear Module: https://www.igus.eu/drylinHTSConfigurator has a 2 mm feed/rotation.
    #   a2 (Porovin AliExpress Rotary Platform: https://www.aliexpress.com/item/1005003488083711.html?spm=a2g0o.order_detail.order_detail_item.2.68fbf19c3s16He)
    #   has a 18:1 gear ratio
    #   a3 (Xeryon Piezo Linear Actuator) is unconfigured

# Controller Parameter
    #   3600 = 1 full mechanical rev

a1_conversion = 1/2                 # 1 rotation lead screw : 2 mm travel
a2_conversion = 1/np.pi * 18        # 1 rotaton = 2*pi rad and 18:1 Gear Reduction (18 stepper rotation / platform rotation)   

motor_ctrl_per_rotation = 3600      # Controller Parameter: 3600 = 1 full mechanical revolution


pub_motor_1 = None
pub_motor_2 = None

def callback(data):

    a1TargetPosition = 0
    a2TargetPosition = 0
    a3TargetPosition = 0

    validReach = True

    a1TargetPosition = data.a1TargetPosition    #   [mm]
    a2TargetPosition = data.a2TargetPosition    #   [rad]
    a3TargetPosition = data.a3TargetPosition    #   [mm]

    rospy.loginfo(a1TargetPosition)
    rospy.loginfo(a2TargetPosition)

    motor_ctrl_1 = MotorCtrlMessage()
    motor_ctrl_2 = MotorCtrlMessage()


    motor_ctrl_1.targetPosition = int(a1TargetPosition*a1_conversion*motor_ctrl_per_rotation)
    motor_ctrl_2.targetPosition = int(a2TargetPosition*a2_conversion*motor_ctrl_per_rotation)
    motor_ctrl_1.operationMode = MotorCtrlMessage.NANOTEC_OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION
    motor_ctrl_2.operationMode = MotorCtrlMessage.NANOTEC_OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION

    rospy.loginfo(motor_ctrl_1.targetPosition)
    rospy.loginfo(motor_ctrl_2.targetPosition)


    # convert a1TargetPosition to Stepper Rotations


    if validReach == True:
        publish_motor_command(motor_ctrl_1, motor_ctrl_2)
    else:
        rospy.loginfo("System cannot reach desired end-effector position.")



    
def conversion_node():

    rospy.loginfo("Started Conversion Node!")

    global pub_motor_1
    global pub_motor_2
    # create the converter node
    rospy.init_node("conversion_node", anonymous=True)
    
    # create subscriber and publisher objects of node
    rospy.Subscriber("motor_position_msg", MotorPositionMessage, callback)
    # 
    pub_motor_1 = rospy.Publisher("/ethercat_master/Nanotec_Motor_1/command", MotorCtrlMessage, queue_size=10)
    pub_motor_2 = rospy.Publisher("/ethercat_master/Nanotec_Motor_2/command", MotorCtrlMessage, queue_size=10)


def publish_motor_command(mc1,mc2):

    pub_motor_1.publish(mc1)
    pub_motor_2.publish(mc2)
    

if __name__ == '__main__': 
    try:
        conversion_node()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass