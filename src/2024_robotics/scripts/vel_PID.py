#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import os

current_state = State()
current_pose = PoseStamped()

# 獲取當前mavros系統狀態
def state_cb(msg):
    global current_state
    current_state = msg

# 獲取當前mavros位置與姿態
def pose_cb(msg):
    global current_pose
    current_pose = msg

# 計算兩點相差距離
def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


# 定義waypoint
def Waypoint(x, y, z):
    set_point = PoseStamped()
    set_point.pose.position.x = x
    set_point.pose.position.y = y
    set_point.pose.position.z = z
    return set_point

# 讀取waypoint檔案
def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            x, y = map(float, line.split())
            waypoints.append(Waypoint(x, y, 1))  # Assuming z = 1 for all waypoints
    return waypoints

# PID控制
def pid_control(target, current, kp, ki, kd, integral, previous_error, dt):
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, integral, error

# 定義waypoint檔案位置並讀取
script_dir = os.path.dirname(__file__)
file_path = os.path.join(script_dir, 'waypoint2.txt')
waypoints = read_waypoints(file_path)

if __name__ == "__main__":
    # 初始化node
    rospy.init_node("vel_PID_py")

    # 定義subscriber，訂閱mavros系統狀態與位置與姿態訊息
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)

    # 定義publisher，發布mavros速度控制命令
    local_setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    # 等待上/解鎖service，並設定此service porxy
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    # 等待模式設置service，並設定此service porxy
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(1000)

    # 檢查node是否正常開啟且無人機是否已經連接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # 初始化線速度為0
    twist = TwistStamped()
    twist.twist.linear.x = 0
    twist.twist.linear.y = 0
    twist.twist.linear.z = 0

    # 發送100次初始速度控制命令，使無人機停止
    for _ in range(100):
        if rospy.is_shutdown():
            break
        local_setpoint_vel_pub.publish(twist)
        rate.sleep()

    # 定義模式設定請求 = OFFBOARD
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # 定義上/解鎖設定請求 = True(解鎖)
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # 紀錄當前時間
    last_req = rospy.Time.now()

    # 定義waypoint索引 = 0 (initialization)
    waypoint_index = 0

    # 定義PID control參數
    kp = 1
    ki = 0.0
    kd = 0.1

    # 初始化PID control 積分變量
    integral_x = 0.0
    integral_y = 0.0
    integral_z = 0.0
    
    # 初始化PID control 誤差變量
    previous_error_x = 0.0
    previous_error_y = 0.0
    previous_error_z = 0.0

    while not rospy.is_shutdown():
        # 檢查無人機是否備設定在OFFBOARD＆解鎖
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        # 檢查無人機當前位置是否到達waypoint，如果到達就設定下個waypoint
        if distance(current_pose.pose.position, waypoints[waypoint_index].pose.position) < 2.0:
            waypoint_index += 1
            if waypoint_index >= len(waypoints):
                waypoint_index = len(waypoints) - 1
        
        # 1000 Hz control loop frequency
        dt = 1.0 / 1000.0  

        # 設定目標點位
        target_x = waypoints[waypoint_index].pose.position.x
        target_y = waypoints[waypoint_index].pose.position.y
        target_z = waypoints[waypoint_index].pose.position.z

        # 設定當前無人機位置
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z

        # 將目標與當前位置傳入PID control，得到速度控制命令
        vel_x, integral_x, previous_error_x = pid_control(target_x, current_x, kp, ki, kd, integral_x, previous_error_x, dt)
        vel_y, integral_y, previous_error_y = pid_control(target_y, current_y, kp, ki, kd, integral_y, previous_error_y, dt)
        vel_z, integral_z, previous_error_z = pid_control(target_z, current_z, kp, ki, kd, integral_z, previous_error_z, dt)

        # 設定並發送速度控制命令
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        twist.twist.linear.z = vel_z

        local_setpoint_vel_pub.publish(twist)
        rate.sleep()
