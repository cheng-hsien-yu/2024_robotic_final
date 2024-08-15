#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
import csv
import os

# 初始化全局变量
ukf_position_data = [None, None, None]
local_position_data = [None, None, None]

script_dir = os.path.dirname(__file__)
file_path = os.path.join(script_dir, 'data.csv')

# 定义回调函数，将数据追加到CSV文件
def ukf_position_callback(data):
    global ukf_position_data
    ukf_position_data = [data.x, data.y, data.z]
    write_to_csv()


def local_position_callback(data):
    global local_position_data
    local_position_data = [data.x, data.y, data.z]
    write_to_csv()


def write_to_csv():
    # 确保所有数据都已初始化
    global file_path
    if None not in ukf_position_data and  None not in local_position_data:
        with open(file_path, 'a', newline='') as file:
            # writer = csv.writer(file)
            # writer.writerow(ukf_position_data + ukf_velocity_data + local_position_data + local_velocity_data)
            writer = csv.writer(file)
            writer.writerow(ukf_position_data  + local_position_data)

def main():
    global file_path
    # 初始化CSV文件并写入表头
    with open(file_path, 'w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(["ukf_pos_x", "ukf_pos_y", "ukf_pos_z",
                    "local_pos_x", "local_pos_y", "local_pos_z"])


    # 初始化ROS节点
    rospy.init_node('data_saver_node')
    
    # 定义订阅者，订阅UKF位置和速度话题
    rospy.Subscriber('/ukf/position', Vector3, ukf_position_callback)
    rospy.Subscriber('/local/position', Vector3, local_position_callback)
    
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
