#!/usr/bin/env python

# Title: Clustering service
# Description: Recieves metacluster locations and auctions them to the ASVs
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import sys
import rospy
import math
import numpy as np
import random
from sklearn import cluster
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#from docker_swarm.msg import *
from docker_swarm.srv import *
import time
import smtplib
from email.mime.text import MIMEText
import csv
sys.path.append('/DataStorage')
import config

number_boats = config.params['asvs']

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

### Euclidean Distance
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


### Send clusters to asvs
def send_clusters_to_asv(boat,clusters,metacluster):
        pose_array = PoseArray()
        pose_array.poses = clusters
        service = '/asv' + str(boat+1)+ '/asv_service'
        rospy.wait_for_service(service)
        try:
                boat_info_rec_service = rospy.ServiceProxy('/asv' + str(boat+1)+ '/asv_service',BoatInfo)
                runtime = []
                runtime.append([get_size(metacluster)+get_size(pose_array)*len(pose_array.poses),0])
                with open("/DataStorage/auction_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)
                response = boat_info_rec_service(metacluster,pose_array)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

### Call Clustering Service
def auctioning(auction_info, boat_info):
        global number_boats
        boats = range(number_boats)     
        assignments = [0]*number_boats
        boat_info_temp = boat_info

        ### Perform Auction Algorithm
        for metacluster in auction_info.metaclusters.poses:
                min_value = 999999
                min_index = 0
                for index in boats:
                        meta_pose = (metacluster.position.x, metacluster.position.y)
                        boat_pose = (boat_info_temp[index].location.position.x,boat_info_temp[index].location.position.y)
                        if abs(distance(meta_pose,boat_pose)) < min_value:
                                min_value = abs(distance(meta_pose,boat_pose))
                                min_index = index
                assignments[min_index] = meta_pose
                boat_info_temp[min_index].location.position.x = 9999999999
        print assignments

        ### Calculate which clusters are in the asv assigned metacluster
        for boat in boats:
                clusters = []
                metacluster = assignments[boat]
                for cluster in auction_info.clusters.poses:
                        min_dist = 999999
                        min_index = 0
                        for index,metacluster in enumerate(auction_info.metaclusters.poses):
                                meta_pose = (metacluster.position.x, metacluster.position.y)
                                cluster_pose = (cluster.position.x, cluster.position.y)
                                if abs(distance(meta_pose,cluster_pose)) < min_dist:
                                        min_dist = abs(distance(meta_pose,cluster_pose))
                                        min_index = index
                        if auction_info.metaclusters.poses[min_index].position.x == assignments[boat][0]and auction_info.metaclusters.poses[min_index].position.y == assignments[boat][1]:
                                clusters.append(cluster)
                send_clusters_to_asv(boat,clusters,metacluster)


        #print auction_info
        #print boat_info

### Service Handler
def handle_recv(req):
        t0 = time.time()
        ### Get Boat info
        boat_info = recv_boat_info()
        ### Perform Auction Algorithm
        auctioning(req, boat_info)
        runtime = []
        runtime.append([time.time()-t0, 0])
        delay = r_delay(0.05,random.random())
        runtime.append([delay, 0])
        runtime.append([get_size(AuctionResponse(True)),0])
        time.sleep(delay)
        with open("/DataStorage/auction_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                writer = csv.writer(f)
                writer.writerows(runtime)

        '''runtime = time.time() - t0
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
        msg = MIMEText("Auction Runtime: " + str(runtime))
        server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg.as_string())
        server.quit()'''
        return AuctionResponse(True)

### Receive People Poses from UAV Service
def recv_metaclusters():
        #global assigns
        print "Starting the auction service"
        rospy.Service('/auctioning', Auction, handle_recv)
        print "Successfully finished the auction service"
        rospy.spin()

def recv_boat_info():
        global number_boats
        responses = []
        for i in range(number_boats):
                service = '/asv' + str(i+1)+ '/asv_service'
                print service
                rospy.wait_for_service(service)
                try:
                        boat_info_rec_service = rospy.ServiceProxy('/asv' + str(i+1)+ '/asv_service',BoatInfo)
                        response = boat_info_rec_service(Pose(),PoseArray())
                        responses.append(response)
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
        return responses

### Main Service Client for auctioner
def auction_service():

        ### Receive Metaclusters
        recv_metaclusters()


if __name__ == '__main__':
        rospy.init_node('auction_service', anonymous=True)
        auction_service()


