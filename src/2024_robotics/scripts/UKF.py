#!/usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from sensor_msgs.msg import Imu
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Vector3
from scipy.linalg import cholesky, LinAlgError

# quaternion --> rotation matrix
def correct_quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
        [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
        [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
    ])

# 預測下一個狀態
def state_transition_function(x, dt):
    pos = x[:3]
    vel = x[3:6]
    acc = x[6:9]
    
    new_pos = pos + vel * dt + 0.5 * acc * dt**2
    new_vel = vel + acc * dt
    
    return np.concatenate((new_pos, new_vel, acc))

# 提取觀測值
def measurement_function(x):
    return x[:3]

# 設定UKF取樣點
points = MerweScaledSigmaPoints(n=9, alpha=0.1, beta=2., kappa=0)

# 建立UKF
ukf = UKF(dim_x=9, dim_z=3, fx=state_transition_function, hx=measurement_function, dt=1/50, points=points)

# 初始化UKF參數
ukf.x = np.zeros(9)
ukf.P = np.eye(9) * 0.1
ukf.R = np.eye(3) * 0.1
ukf.Q = np.eye(9) * 0.1

def is_positive_definite(matrix):
    try:
        np.linalg.cholesky(matrix)
        return True
    except np.linalg.LinAlgError:
        return False

# 獲取IMU參數
def imu_cb(data):
    # 提取IMU quaternion
    quaternion = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ]
    
    # quaternion to rotation matrix
    rotation_matrix = correct_quaternion_to_rotation_matrix(quaternion)

    # 提取IMU加速度值，並將小於0.4的值判定為noise
    edge = 0.4
    linear_acceleration_body = np.array([
        0 if -edge < data.linear_acceleration.x < edge else data.linear_acceleration.x,
        0 if -edge < data.linear_acceleration.y < edge else data.linear_acceleration.y,
        0 if -edge < data.linear_acceleration.z < edge else data.linear_acceleration.z
    ])
    
    # 將IMU加速度值轉換到世界座標系
    linear_acceleration_world = np.dot(rotation_matrix, linear_acceleration_body)
    #gravity = np.array([0, 0, 9.81])
    linear_acceleration_motion = linear_acceleration_world 
    
    ukf.x[6:9] = linear_acceleration_motion

    # 更新UKF
    ukf.predict()

# 獲取AprilTag位置
def apriltag(x, y, z):
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    return p

apriltags = [apriltag(0, 0, 0),
             apriltag(5, 0, 0),
             apriltag(10.5, 0, 0),
             apriltag(10.5, 3, 0),
             apriltag(5, 3, 0),
             apriltag(1, 3, 0),
             apriltag(1, 6, 0),
             apriltag(6, 6, 0),
             apriltag(np.nan, np.nan, np.nan),
             apriltag(np.nan, np.nan, np.nan),
             apriltag(12, 6, 0)]

# 透過apriltag去測量無人機位置
def tag_detections_callback(msg):
    for detection in msg.detections:
        # 獲取AprilTag相對於相機的位置（rotation matrix）＆方向（quaternion）
        tag_id = detection.id[0]
        position = detection.pose.pose.pose.position
        orientation = detection.pose.pose.pose.orientation
        
        rel_pos_b = np.array([position.x, position.y, position.z])
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        R = correct_quaternion_to_rotation_matrix(q)

        # 將相對位置從body frame轉換到世界座標系
        rel_pos = np.dot(R, rel_pos_b)

        # 檢查tag_id是否在有效範圍內
        if tag_id < len(apriltags):
            tag_pos = np.array([apriltags[tag_id].pose.position.x,
                                apriltags[tag_id].pose.position.y,
                                apriltags[tag_id].pose.position.z])
            
            # 將相對位置 + AprilTag絕對座標 ＝ 無人機測量位置（世界座標系）
            measurement = rel_pos + tag_pos
            # rospy.loginfo("AprilTag detection: ID = %d\n measurement = %s", tag_id, measurement)

            # 更新UKF
            ukf.update(measurement)
        else:
            rospy.logwarn("Detected AprilTag with ID %d is out of range.", tag_id)


local_pos_msg = Vector3()

def local_position_callback(data):
    global local_pos_msg
    local_pos_msg.x = data.pose.position.x
    local_pos_msg.y = data.pose.position.y
    local_pos_msg.z = data.pose.position.z

def main():
    # 初始化node
    rospy.init_node('ukf_estimation_node')
    
    # 定義subscriber，訂閱imu, apriltag
    rospy.Subscriber('/mavros/imu/data', Imu, imu_cb)
    rospy.Subscriber('/camera_down/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    rospy.Subscriber('/camera_front/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_position_callback)
    
    # 定義publisher，發佈速度與位置
    velocity_pub = rospy.Publisher('/ukf/velocity', Vector3, queue_size=10)
    position_pub = rospy.Publisher('/ukf/position', Vector3, queue_size=10)
    local_position_pub = rospy.Publisher('/local/position', Vector3, queue_size=10)
    
    rate = rospy.Rate(50)  # 50 Hz
    
    # 進行UKF控制，透過上述函式的測量值與估計值進行更新
    while not rospy.is_shutdown():
        pos = ukf.x[:3]
        vel = ukf.x[3:6]
        
        pos_msg = Vector3()
        pos_msg.x = pos[0]
        pos_msg.y = pos[1]
        pos_msg.z = pos[2]
        
        vel_msg = Vector3()
        vel_msg.x = vel[0]
        vel_msg.y = vel[1]
        vel_msg.z = vel[2]
        
        position_pub.publish(pos_msg)
        velocity_pub.publish(vel_msg)
        local_position_pub.publish(local_pos_msg)
        
        rospy.loginfo("UKF position: \n x: %s \n y: %s \n z: %s", pos[0], pos[1], pos[2])
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
