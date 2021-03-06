# code to interface the hardware
# TODO - Integrate JointTrajectoryAction server
#      - Enable & disable torque service
#      - Set PID of joints as service or param
import rospy
import herkulex
import math
from sensor_msgs.msg import JointState
num_joints = 4
herkulex.connect("/dev/ttyUSB0",115200)
herkulex.torque_off(1)
herkulex.torque_off(2)



def joint_state_publisher():
    pub = rospy.Publisher('joint_states', JointState,queue_size = 5)
    rospy.init_node('sr_jnt_stt')
    
    r = rospy.Rate(300)
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = num_joints * [0.0]
        msg.velocity = num_joints * [0.0]
        #msg.effort = num_joints * [0.0]
        msg.name =  ['joint1', 'joint2', 'joint3', 'gripper_con']
        msg.position[0] = math.radians(herkulex.get_servo_angle(1))
        msg.position[1] = math.radians(herkulex.get_servo_angle(2))
        msg.position[2] =0.0
        msg.position[3] =0.0
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException: pass
