#!/usr/bin/env python

# Title: Clustering service
# Description: Recieves cluster locations and metaclusters them
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
import skfuzzy as fuzz
import smtplib
import time
from email.mime.text import MIMEText
import csv
sys.path.append('/DataStorage')
import config

num_boats = config.params['asvs']

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

def cmeans_clustering(data):
        global num_boats
        clusters = data.clusters.poses
        location_array = []
        if(data != []):
                ### Store clusters x&y poses into array
                #location_array = [(data.clusters[0].pose.position.x,data.clusters[0].pose.position.y)]
                for i in range(len(data.clusters.poses)):
                        location_array.append((data.clusters.poses[i].position.x,data.clusters.poses[i].position.y))

                ### K-means clustering on Clusters
                k = num_boats
                cntr, u, u0, d, jm, p, fpc = fuzz.cluster.cmeans(np.asarray(location_array).T, k, 2, error=0.005, maxiter=5000, init=Non$
                centroids = []
                for pt in cntr:
                        centroids.append((pt[0],pt[1])) 

                cluster_membership = np.argmax(u, axis=0)
                labels = u

        return centroids,labels,location_array

### Call Clustering Service
def metaclustering(clusters):
        metaclusters, labels, locations = cmeans_clustering(clusters)

        rospy.wait_for_service('/auctioning')
        try:
                master_pose_rec_service = rospy.ServiceProxy('/auctioning',Auction)

                poses = []
                for metacluster in metaclusters:
                        pose = Pose()
                        pose.position.x = metacluster[0]
                        pose.position.y = metacluster[1]
                        poses.append(pose)

                metaclusters_poses = PoseArray()
                metaclusters_poses.poses = poses
                print type(metaclusters_poses)
                print type(clusters.clusters)

                runtime = []
                runtime.append([get_size(metaclusters_poses)*len(metaclusters_poses.poses)+get_size(clusters.clusters.poses)*len(cluster$
                with open("/DataStorage/metaclustering_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                        writer = csv.writer(f)
                        writer.writerows(runtime)
                master_pose_rec_service(metaclusters_poses,clusters.clusters)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

### Service Handler
def handle_recv(req):
        t0 = time.time()
        metaclustering(req)
        runtime = []
        runtime.append([time.time()-t0, 0])
        delay = r_delay(0.05,random.random())
        runtime.append([delay, 0])
        runtime.append([get_size(ClusterLocsResponse(True)),0])
        time.sleep(delay)
        with open("/DataStorage/metaclustering_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                writer = csv.writer(f)
                writer.writerows(runtime)

        '''runtime = time.time() - t0
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
        msg = MIMEText("Metaclustering Runtime: " + str(runtime))
        server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg.as_string())
        server.quit()'''
        return ClusterLocsResponse(True)

### Receive People Poses from UAV Service
def recv_clusters():
        #global assigns
        print "Starting the metaclustering service"
        rospy.Service('/metaclustering', ClusterLocs, handle_recv)
        print "Successfully finished the metaclustering service"
        rospy.spin()



### Main Service Client for UAV
def metaclustering_service():

        ### Receive People Poses from UAV Service
        recv_clusters()


if __name__ == '__main__':
        rospy.init_node('metaclustering_service', anonymous=True)
        metaclustering_service()

