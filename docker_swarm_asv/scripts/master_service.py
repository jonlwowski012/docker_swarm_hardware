#!/usr/bin/env python

# Title: Master service
# Description: Recieves people locations from the UAV
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import sys
import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#from docker_swarm.msg import *
from docker_swarm.srv import *
import smtplib
import time
from email.mime.text import MIMEText
import csv
import time
sys.path.append('/DataStorage')
import config

### Keeps track of # of UAVs that have sent poses
uav_count = 0
total_uav = config.params['uavs']
glob_poses = PoseArray()

### Calculate Size of Object
def get_size(obj, seen=None):
        """Recursively finds size of objects"""
        size = sys.getsizeof(obj)
        if seen is None:
                seen = set()
        obj_id = id(obj)
        if obj_id in seen:
                return 0
        # Important mark as seen *before* entering recursion to gracefully handle
        # self-referential object
        seen.add(obj_id)
        if isinstance(obj, dict):
                size += sum([get_size(v, seen) for v in obj.values()])
                size += sum([get_size(k, seen) for k in obj.keys()])
        elif hasattr(obj, '__dict__'):
                size += get_size(obj.__dict__, seen)
        elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
                size += sum([get_size(i, seen) for i in obj])
        return size

### Calculate Time Delay
def r_delay(sigma, r):
        delay = sigma*math.sqrt(-2*math.log(r))
        return delay

### Call Clustering Service
def clustering(pos):
        pose_arr = PoseArray()
        pose_arr.poses = pos.poses
        rospy.wait_for_service('/clustering')
        try:
                master_pose_rec_service = rospy.ServiceProxy('/clustering',PeopleLocs)
                runtime = []
                runtime.append([get_size(pos)*len(pos.poses),0])
                with open("/DataStorage/master_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)
                master_pose_rec_service(pos)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
### Service Handler
def handle_recv(req):
        t0 = time.time()
        global uav_count
        global glob_poses
        glob_poses.poses= glob_poses.poses+req.poses.poses

        #glob_poses = req.poses
        uav_count += 1

        # Once all poses are recieved call clustering service
        if uav_count >= total_uav:
                clustering(glob_poses)
                #runtime = time.time() - t0
                '''server = smtplib.SMTP('smtp.gmail.com', 587)
                server.starttls()
                server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
                msg = MIMEText("Master Runtime: " + str(runtime))
                server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg.as_string())
                server.quit()'''
                delay = r_delay(0.05,random.random())
                runtime = []
                runtime.append([time.time()-t0, 0])
                runtime.append([delay, 0])
                runtime.append([get_size(PeopleLocsResponse(True)),0])
                time.sleep(delay)

                with open("/DataStorage/master_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)

        return PeopleLocsResponse(True)

### Recieve People Poses from UAV Service
def recv_poses():
        #global assigns
        print "Starting the master service"
        rospy.Service('/master_poses_receiver', PeopleLocs, handle_recv)
        print "Successfully finished the master service"
        rospy.spin()

### Main Service Client for UAV
def master_service():

        ### Recieve People Poses from UAV Service
        people_locs = recv_poses()


if __name__ == '__main__':
        rospy.init_node('master_service', anonymous=True)
        master_service()


