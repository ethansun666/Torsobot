#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

import message_filters
from std_msgs.msg import Int32, Float32

rospy.init_node('message_sync', anonymous=False)

speed_desired = 0.5 # desired wheel speed in rpm
angle_desired = 0.0 # desired angle - 0
k_p_angle = 4*480.0/90.0 # propotional gain for angle control
k_i_angle = k_p_angle/4. # integral gain for angle control
k_d_angle = 1 # derivatibe gain for angle control
k_p_speed = 15 # proportional gain for speed control (60)
k_i_speed = 35 # integral gain for speed control(30)

time_old= 0.0
time_current = rospy.get_rostime()

def update_current_wheel_speed(msg_in):
    global current_wheel_speed
    current_wheel_speed = msg_in.data
    
def update_imu_angle(msg_in):
    global current_imu_angle
    current_imu_angle = msg_in.data
def PID_control(IMU_message,Encoder_message):
    
    global current_wheel_speed
    global current_imu_angle
    speed_error_cum = 0.0
    
    
    while True:
        
        #### time update
        time_old = time_current # set previous time reading
        time_current = rospy.get_rostime() # set current time reading
        dt = time_current - time_old # time step
        
        # P
        speed_error = speed_desired - current_wheel_speed
        
        # I
        speed_error_cum += speed_error * dt
        
        # Effort
        # not sure what speed_direction_comp is used for
        angle_desired = 1 * (k_p_speed * speed_error + k_i_speed * speed_error_cum)
    

def message_sync():
    IMU_message = message_filters.Subscriber('/IMU_angle', Float32)
    Encoder_message = message_filters.Subscriber('/Encoder_data', Float32)
    
    sync = message_filters.ApproximateTimeSynchronizer([IMU_message,Encoder_message],queue_size = 1,slop = 0.05) #what happens if queue size is not one and what should the slop be
    sync.registerCallback(PID_control)
    
    rospy.spin()

if __name__ == "__main__": 
    try: 
        message_sync()
    except rospy.ROSInterruptException:
        pass