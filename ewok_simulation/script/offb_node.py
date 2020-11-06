#!/usr/bin/env python

import rospy
import mavros
import signal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# callback method for state sub
mavros.set_namespace()
current_state = State() 
local_current_pos = Odometry()
real_target = PoseStamped()
offb_set_mode = SetMode
def state_cb(bad):
    global current_state
    current_state = bad

def cur_pos(data_local):
    global local_current_pos
    local_current_pos = data_local

def target_pos(target_data):
    global real_target
    real_target = target_data

local_pos_pub = rospy.Publisher("/quadrotor/mavros/setpoint_position/local", PoseStamped, queue_size=10)
local_pos_sub = rospy.Subscriber("/quadrotor/mavros/global_position/local", Odometry, cur_pos)
target_pos_sub = rospy.Subscriber("/quadrotor/command/pose", PoseStamped, target_pos)
state_sub = rospy.Subscriber("/quadrotor/mavros/state", State, state_cb)
arming_client = rospy.ServiceProxy("/quadrotor/mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy("/quadrotor/mavros/set_mode", SetMode) 

pose = PoseStamped()

def position_control():
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(real_target)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(real_target)
        rate.sleep()
        #print(local_current_pos.pose.pose.position.z)
        #print(real_target.pose.position.z)

if __name__ == '__main__':
    try:
        rospy.init_node('offb_node', anonymous=True)
        position_control()

    except rospy.ROSInterruptException:
        pass
