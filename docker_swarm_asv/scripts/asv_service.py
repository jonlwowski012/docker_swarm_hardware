#!/usr/bin/env python

# Title: ASV service
# Description: Sends boat info to auctioner and recieves cluster assignments
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import smtplib
import sys
import subprocess
import rospy
import rosnode
import math
import numpy as np
import random
import time
from threading import Thread
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#from docker_swarm.msg import *
from docker_swarm.srv import *
from email.mime.text import MIMEText
import csv
import os

boat_speed = 0.
boat_capacity = 0.
boat_location = Pose()
metacluster = Pose()
clusters = PoseArray()


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
def auction_assignment():
	rospy.wait_for_service('/clustering')
	try:
		master_pose_rec_service = rospy.ServiceProxy('/clustering',PeopleLocs)
		runtime = []
		runtime.append([get_size(pos)*len(pos.poses),0])
		with open("/DataStorage/asv_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)
		master_pose_rec_service(pos)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

### Service Handler
def handle_boatinfo(req):
	global boat_speed
	global boat_capacity
	global boat_location
	global metacluster
	global clusters

	if req.metacluster.position.x == 0. and req.metacluster.position.y == 0.:
		boat_info = BoatInfoResponse()
		boat_info.speed = boat_speed
		boat_info.capacity = boat_capacity
		boat_info.location = boat_location
		return boat_info
	else:
		metacluster = req.metacluster
		clusters = req.clusters
		boat_info = BoatInfoResponse()
		boat_info.speed = boat_speed
		boat_info.capacity = boat_capacity
		boat_info.location = boat_location
		return boat_info

### Call TSP Service
def call_tsp():
	print rospy.get_name()
	t0 = time.time()
	global clusters
	global metacluster
	global boat_location
	while not rospy.is_shutdown():
		if metacluster.position.x == 0 and metacluster.position.y == 0:
			time.sleep(1)
		else:
			rospy.wait_for_service('/traveling_salesman_service')
			try:
				tsp_service = rospy.ServiceProxy('/traveling_salesman_service',TSP)
				path = tsp_service(boat_location,clusters)
				runtime = []
				runtime.append([time.time()-t0, 0])
				delay = r_delay(0.05,random.random())
				runtime.append([delay, 0])
				runtime.append([-1,0])
				path_arr = []
				for loc in path.path.poses:
					path_arr.append([loc.position.x,loc.position.y])

				with open("/DataStorage/asv_path" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
    					writer = csv.writer(f)
    					writer.writerows(path_arr)

				with open("/DataStorage/asv_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                                        print runtime
					writer = csv.writer(f)
                                        writer.writerows(runtime)

				time.sleep(delay)
				'''server = smtplib.SMTP('smtp.gmail.com', 587)
				server.starttls()
				server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
				msg = ''.join(str(e) for e in path_arr)
				print msg
				server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg)
				server.quit()
				time.sleep(1)
				server = smtplib.SMTP('smtp.gmail.com', 587)
				server.starttls()
				server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
				msg = MIMEText("ASV Runtime: " + str(runtime))
				server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg.as_string())
				server.quit()'''
				rosnode.kill_nodes([rospy.get_name()]) 
				break
			except rospy.ServiceException, e:
        			print "Service call failed: %s"%e
	#subprocess.Popen(['rosnode','kill',rospy.get_name()])
	#os.kill(os.getpid(), 9)
	rospy.signal_shutdown('killing') 

### Receive People Poses from UAV Service
def boat_info():
	#global assigns	
	print "Starting the asv service"
	thread= Thread(target = call_tsp)
	thread.start()
	rospy.Service('asv_service', BoatInfo, handle_boatinfo)
	print "Successfully finished the asv service"
	rospy.spin()

### Main Service Client for UAV
def asv_service():
	global boat_speed
	global boat_capacity
	global boat_location
	boat_speed = random.uniform(.2,1.0)
	boat_capacity = random.uniform(10,150)
	boat_location.position.x = random.uniform(-200,200)
	boat_location.position.y = random.uniform(-200,200)
	### Send boat info
	sent_flag = boat_info()
	

if __name__ == '__main__':
	rospy.init_node('asv_service', anonymous=True)
	asv_service()
	

