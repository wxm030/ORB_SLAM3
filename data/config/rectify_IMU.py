import os
import sys
import shutil
import re
import json

import pandas as pd
import numpy as np
import pyquaternion
import csv
import yaml
import cv2





if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("Usage: {0}  imu_csv  imu_intrinsic_yaml".format(sys.argv[0]))
        exit(-1)


    #read imu intrinsics params
    cv_file = cv2.FileStorage(sys.argv[2], cv2.FILE_STORAGE_READ)
    gyro_M = cv_file.getNode("Gyroscope.M").mat()
    gyro_A = cv_file.getNode("Gyroscope.A").mat()
    gyro_C = cv_file.getNode("Gyroscope.C_gyro_i").mat()
    acc_M = cv_file.getNode("Accelerometer.M").mat()

    bias_wx = cv_file.getNode("IMU.Bias_wx").real()
    bias_wy = cv_file.getNode("IMU.Bias_wy").real()
    bias_wz = cv_file.getNode("IMU.Bias_wz").real()
    bias_ax = cv_file.getNode("IMU.Bias_ax").real()
    bias_ay = cv_file.getNode("IMU.Bias_ay").real()
    bias_az = cv_file.getNode("IMU.Bias_az").real()

    bias_w = np.array([bias_wx,bias_wy,bias_wz])
    bias_a = np.array([bias_ax,bias_ay,bias_az])
    cv_file.release()


    bias_a_rectify = np.dot(np.linalg.inv(acc_M),bias_a.T)
    bias_w_rectify =  np.dot(np.linalg.inv(gyro_M),bias_w.T)
    print("bias_a_rectify:")
    print(bias_a_rectify)
    print("bias_w_rectify:")
    print(bias_w_rectify)

    #out imu data csv file
    csv_file_out = open('data.csv', 'w')
    header = ['#timestamp [ns]', 'w_RS_S_x [rad s^-1]','w_RS_S_y [rad s^-1]','w_RS_S_z [rad s^-1]','a_RS_S_x [m s^-2]','a_RS_S_y [m s^-2]','a_RS_S_z [m s^-2]']
    writer = csv.DictWriter(csv_file_out, fieldnames=header)
    writer.writeheader()

    #read imu csv data
    csvFile = open(sys.argv[1], "r")
    imu_datas = dict()
    reader = csv.DictReader(csvFile)
    for row in reader:
        timestamp = row["#timestamp [ns]"]
        wx = row["w_RS_S_x [rad s^-1]"]
        wy = row["w_RS_S_y [rad s^-1]"]
        wz = row["w_RS_S_z [rad s^-1]"]
        ax = row["a_RS_S_x [m s^-2]"]
        ay = row["a_RS_S_y [m s^-2]"]
        az = row["a_RS_S_z [m s^-2]"]
        w = np.array([wx,wy,wz])
        a = np.array([ax,ay,az])

        a_rectify = np.dot(np.linalg.inv(acc_M).astype(float),a.T.astype(float))
        a_tmp = np.dot(np.linalg.inv(acc_M).astype(float), (a.T.astype(float) - bias_a.T.astype(float)))  
        w_rectify = np.dot(np.linalg.inv(gyro_M).astype(float), (w.T.astype(float) - np.dot(gyro_A,a_tmp).astype(float)))

   
        writer.writerow({'#timestamp [ns]': timestamp, 
                         'w_RS_S_x [rad s^-1]': w_rectify[0],
                         'w_RS_S_y [rad s^-1]': w_rectify[1],
                         'w_RS_S_z [rad s^-1]': w_rectify[2],
                         'a_RS_S_x [m s^-2]': a_rectify[0],
                         'a_RS_S_y [m s^-2]': a_rectify[1],
                         'a_RS_S_z [m s^-2]': a_rectify[2]
                        })
        
        

