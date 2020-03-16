#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

import message_filters
from std_msgs.msg import Int32, Float32
from pololu_drv8835_rpi import motors


rospy.init_node('message_sync', anonymous=False)

speed_desired = 0.5 # desired wheel speed in rpm
angle_desired = 0.0 # desired angle - 0
k_p_angle = 4*480.0/90.0 # propotional gain for angle control
k_i_angle = k_p_angle/4. # integral gain for angle control
k_d_angle = 1 # derivatibe gain for angle control
k_p_speed = 15 # proportional gain for speed control (60)
k_i_speed = 35 # integral gain for speed control(30)



def drive_motor(speed): # send speed command to motor
    max_speed = 380
    if speed > max_speed:
        drive_speed = max_speed
    elif speed < -max_speed:
        drive_speed = -max_speed
    else:
        drive_speed = round(speed)
    motors.motor1.setSpeed(int(drive_speed))
    return drive_speed

def PID_control(IMU_message,Encoder_message):
    
    global current_wheel_speed
    global current_imu_angle
    global speed_error_cum
    global angle_error_cum
    global time_current

    current_wheel_speed = Encoder_message.data
    angle_prev = current_imu_angle
    current_imu_angle = IMU_message.data
    
        
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
        
    if current_imu_angle <= 90 and current_imu_angle >= -90:
        # P
        angle_error = angle_desired - current_imu_angle
        # I
        angle_error_cum += angle_error*dt
        # D
        angle_diff = (angle_prev - current_imu_angle)/dt
        
        # Effort
        motor_output = -1*(k_p_angle*angle_error + k_i_angle*angle_error_cum + k_d_angle*angle_diff)
        drive_motor(motor_output)

def message_sync():

    global speed_error_cum
    global angle_error_cum
    global time_current
    speed_error_cum = 0.0
    angle_error_cum = 0.0
    
    time_current = rospy.get_rostime()
    
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