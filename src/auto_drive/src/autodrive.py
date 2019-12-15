#!/usr/bin/env python
import rospy, time
from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from TrafficLightDetector import *
from motordriver import MotorDriver
from imuread import ImuRead

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.imu = ImuRead('/diagnostics')

    def trace(self):
        obs_l, obs_m, obs_r, obs_s1, obs_s2 = self.obstacle_detector.get_distance()
        line_theta, left, right = self.line_detector.detect_lines()
        r, p, y = self.imu.get_data()
        angle = self.steer(line_theta, left, right)
        speed = self.accelerate(angle, line_theta, left, right, obs_s1, obs_s2, p)
        print("angle : ", int(angle), "speed : ", int(speed), "theta : ", int(line_theta), "distance : ", int(obs_s1), int(obs_s2), "pitch : ", p)
        self.driver.drive(angle + 90 - 1.2, speed)
        
    def steer(self, theta, left, right):
        mag_steer = 0.0
        mag_calib = 0.7
        if -0.8 <= theta <= 0.0:
            mag_steer = 8.0 + mag_calib
        elif 0.0 <= theta <= 0.8:
            mag_steer = 15.0 + mag_calib    
        elif -4 <= theta <= 4:
            mag_steer = 2.5 + mag_calib
        elif 4 <= theta <= 15:
            mag_steer = 1.75 + mag_calib
        elif -15 <= theta <= -4:
            mag_steer = 1.86 + mag_calib       
        elif -15 <= theta <= 15:
            mag_steer = 1.75 + mag_calib
        elif theta >= 20:
            mag_steer = 3.00 + mag_calib
        elif theta <= -20:
            mag_steer = 1.80 + mag_calib
        else:
            mag_steer = 1.50 + mag_calib
  
        angle = theta * mag_steer
        return angle

    def accelerate(self, angle, theta, left, right, obs_s1, obs_s2, pitch):
        origin = 130
        '''
        if (obs_s1 <= 70) and (obs_s2 <= 70):
            origin = 120
        '''
        if pitch < -3:
            origin = 140
        elif 3 < pitch:
            origin = 120
        mag_decel = min(abs(theta)/2, 10) 
        speed = origin - mag_decel
        return speed

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
