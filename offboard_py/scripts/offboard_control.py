#! /usr/bin/env python

from math import *
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest



current_state = State()
current_pose = PoseStamped()
pose_msg = PoseStamped()
itration = 0


def state_cb(msg):
    global current_state
    current_state = msg


def position_cb(msg):
    global current_pose, pose_msg
    pose_msg = msg
    current_pose.pose.position.x = pose_msg.pose.position.x
    current_pose.pose.position.y = pose_msg.pose.position.y
    current_pose.pose.position.z = pose_msg.pose.position.z


def circle():
    List = []
    res = 1000
    radius = 2
    for a in range(res):
        x = radius * sin(2*pi*(a/res))
        y = radius * cos(2*pi*(a/res))
        List.append((x,y))
    return List


def infinity():
    List = []
    res = 1000
    radius = 2
    const = pi * 0.5
    for a in range(res):
        x = radius * cos(2*pi*(a/res) - const)
        y = radius * cos(2*pi*(a/res) - const) * sin(2*pi*(a/res) - const)
        List.append((x,y))
    return List    


def update_pos():
    global itration,current_pose,curr_target_pose
    x = current_pose.pose.position.x
    y = current_pose.pose.position.y
    error = 0.1
    try:
        if abs(x - List[itration][0]) <= error and abs(y - List[itration][1])<= error:
            target_pose = List[itration]
            itration += 1
            # print(itration)
            return target_pose
        else:
            return curr_target_pose
    except(IndexError):
        return [0,0]

List = infinity()
curr_target_pose = List[0]
# print(List)


if __name__ == "__main__":
    rospy.init_node("offboard_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    current_position = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=position_cb)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 1
    pose.pose.position.y = 1
    pose.pose.position.z = 1
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    Position_Listx = []
    Position_Listy = []
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        # print("____")
        local_pos_pub.publish(pose)
        curr_target_pose = update_pos()
        if curr_target_pose == [0, 0]:
            break
        Position_Listx.append(current_pose.pose.position.x)
        Position_Listy.append(current_pose.pose.position.y)
        pose.pose.position.x = curr_target_pose[0]
        pose.pose.position.y = curr_target_pose[1]
        # print(current_pose)
        rate.sleep()
    # print(Position_Listx)
    # print(Position_Listy)
    # print("______________")
    
    figure, axis = plt.subplots(1, 2) 
    axis[0].plot(Position_Listx,Position_Listy)
    axis[0].set_title('Actual location data')
    x = []
    y = []
    for a in List:
        x.append(a[0])
        y.append(a[1])
    axis[1].plot(x,y)
    axis[1].set_title('Desired Trajectory')
    # plt.legend() 
    plt.show()
