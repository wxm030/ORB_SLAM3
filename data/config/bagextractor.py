#!/usr/bin/env python
print "importing libraries"

import cv2
import csv
import os
import sys
import argparse
import rosbag
from cv_bridge import CvBridge

try:
    import cv
    png_flag = cv.CV_IMWRITE_PNG_COMPRESSION
except ImportError:    
    png_flag = cv2.IMWRITE_PNG_COMPRESSION


#setup the argument list
parser = argparse.ArgumentParser(description='Extract a ROS bag containing a image and imu topics.')
parser.add_argument('--bag', metavar='bag', help='ROS bag file')
parser.add_argument('--image-topics',  metavar='image_topics', nargs='+', help='Image topics %(default)s')
parser.add_argument('--imu-topics',  metavar='imu_topics', nargs='+', help='Imu topics %(default)s')
parser.add_argument('--output-folder',  metavar='output_folder', nargs='?', default="output", help='Output folder %(default)s')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

#parse the args
parsed = parser.parse_args()

if parsed.image_topics is None and parsed.imu_topics is None:
    print "ERROR: Need at least one camera or IMU topic."
    sys.exit(-1)

#create output folder
try:
  os.makedirs(parsed.output_folder)
except:
  pass



cam_topics = parsed.image_topics
imu_topics = {"/imu0"}#parsed.imu_topics



#extract imu data
iidx = 0

filename = "data.csv"
os.makedirs("{0}/imu0".format(parsed.output_folder))  
with open( "{0}/{1}".format(parsed.output_folder + "/imu0", filename), 'wb') as imufile:
    spamwriter = csv.writer(imufile, delimiter=',')
    spamwriter.writerow(["#timestamp [ns]","w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z[rad s^-1]", "a_RS_S_x[m s^-2]", "a_RS_S_y [m	s^-2]", "a_RS_S_z [m s^-2]"])
    #spamwriter.writerow(["timestamp", "omega_x", "omega_y", "omega_z", "alpha_x", "alpha_y", "alpha_z"])

    for topic, msg, t in rosbag.Bag(parsed.bag).read_messages():
        if topic in imu_topics:
            omega = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            alpha = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            timestamp_int = t
            spamwriter.writerow([timestamp_int, omega[0],omega[1],omega[2], alpha[0],alpha[1],alpha[2] ])


#extract cam0 images to folder
cidx = 0
os.makedirs("{0}/cam{1}".format(parsed.output_folder, cidx))  
os.makedirs("{0}/cam{1}/data".format(parsed.output_folder, cidx))  


bridge = CvBridge()
cam_data_csv = "data.csv"



f1 = open( "{0}/timestamps.txt".format(parsed.output_folder), 'wb')
with open( "{0}/cam{1}/{2}".format(parsed.output_folder, cidx, cam_data_csv), 'wb') as cam_csv:
    spamwriter_cam = csv.writer(cam_csv, delimiter=',')
    spamwriter_cam.writerow(["timestamp", "filename"])
    for topic, msg, t in rosbag.Bag(parsed.bag).read_messages():
        if topic in cam_topics:
            timestamp = msg.header.stamp
            params = list()
            params.append(png_flag)
            params.append(0) #0: loss-less  
            filename_img = "{0}{1:09d}.png".format(timestamp.secs, timestamp.nsecs)

            spamwriter_cam.writerow([t, filename_img ])
            
            f1.write("{0}{1:09d}".format(timestamp.secs, timestamp.nsecs))
            f1.write("\n")
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite( "{0}/cam{1}/data/{2}".format(parsed.output_folder, cidx, filename_img), cv_img, params )
f1.close()
cam_csv.close()



