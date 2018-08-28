#!/usr/bin/env python

# Title: UAV client
# Description: Randomly generates peoples poses and sends them to the master
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import smtplib
import sys
import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#from docker_swarm.msg import *
from docker_swarm.srv import *
import csv
import time
sys.path.append('/DataStorage')
import config

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

### Sends poses of all people to master via ROS Service
def send_poses_to_master(poses):
        pose_arr = PoseArray()
        pose_arr.poses = poses
        delay = r_delay(0.05,random.random())
        time.sleep(delay)       
        rospy.wait_for_service('/master_poses_receiver')
        try:
                master_pose_rec_service = rospy.ServiceProxy('/master_poses_receiver',PeopleLocs)
                runtime = []
                runtime.append([get_size(pose_arr)*len(pose_arr.poses),0])
                with open("/DataStorage/uav_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)
                master_pose_rec_service(pose_arr)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

### Randomly generate peoples poses
def gen_poses():
        people_locs = []
        num_people = config.params['people_per_uav']
        while num_people > 0:
                pose = Pose()
                pose.position.x = random.uniform(-200,200)
                pose.position.y = random.uniform(-200,200)
                num_people -= 1
                people_locs.append(pose)
        path_arr = []
        for loc in people_locs:
                path_arr.append([loc.position.x,loc.position.y])
        with open("/DataStorage/uav_people" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                writer = csv.writer(f)
                writer.writerows(path_arr)


        '''server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
        msg = ''.join(str(e) for e in path_arr)
        server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg)
        server.quit()'''
        return people_locs


### Main Service Client for UAV
def uav_service():

        ### Randomly generate peoples poses
        people_locs = gen_poses()

        ### Send People Poses to master
        send_poses_to_master(people_locs)


if __name__ == '__main__':
        rospy.init_node('uav_client', anonymous=True)
        uav_service()

