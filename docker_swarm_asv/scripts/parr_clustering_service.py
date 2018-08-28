#!/usr/bin/env python

# Title: Clustering service
# Description: Recieves people locations from the master and clusters them
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
from mpi4py import MPI

### MPI Functions   ####
comm = MPI.COMM_WORLD
rank = comm.Get_rank()
size = comm.Get_size()
tf=0
t0=0

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

### Call Metaclustering Service
def metaclustering(clusters):
                cluster_poses = []
                for cluster in clusters:
                                pose = Pose()
                                pose.position.x = cluster[0]
                                pose.position.y = cluster[1]
                                cluster_poses.append(pose)
                pose_arr = PoseArray()
                pose_arr.poses = cluster_poses
                rospy.wait_for_service('/metaclustering')
                try:
                                master_pose_rec_service = rospy.ServiceProxy('/metaclustering',ClusterLocs)
                                runtime = []
                                runtime.append([get_size(pose_arr)*len(pose_arr.poses),0])
                                with open("/DataStorage/clustering_runtime_send" + rospy.get_namespace().replace("/", '')  + ".csv", "wb$
                                                writer = csv.writer(f)
                                                writer.writerows(runtime)
                                master_pose_rec_service(pose_arr)
                except rospy.ServiceException, e:
                                print "Service call failed: %s"%e

def clustering(poses):
                global t0
                global tf
                t0 = time.time()
                data = poses.poses.poses
                location_array = []
                for i in range(len(data)):
                                location_array.append((data[i].position.x,data[i].position.y))
                location_array = comm.bcast(location_array, root=0)
                flag = False
                inertia = 100000
                ks = range(1,size+1)
                ### while the average cluster radius is greater than 10m
                while((inertia/len(location_array))>= 20 or flag == False):
                                ks_copy = list(ks)
                                data = comm.scatter(ks_copy, root=0)
                                ### Calculate Clusters
                                kmeans = cluster.KMeans(init='k-means++', n_init=10, n_clusters=int(data))
                                kmeans.fit(location_array)
                                inertia = kmeans.inertia_
                                #print "inertia for process ", rank, ": ", inertia
                                min_inertia = comm.reduce(inertia, op=MPI.MIN, root=0)
                                inertia = min_inertia
                                if rank == 0:
                                        print "inertia: ", min_inertia
                                flag = True

                                for index_k in range(len(ks)):
                                        ks[index_k] += 1
                                #ks = [x+1 for x in ks]

                if rank == 0:
                        ### Calculate Clusters
                        kmeans = cluster.KMeans(init='k-means++', n_init=10, n_clusters=int(data))
                        kmeans.fit(location_array)
                        centroids_temp = kmeans.cluster_centers_
                        labels = kmeans.labels_
                        inertia = kmeans.inertia_

                        ### Calculate Radius for Cluster Msg
                        radius = inertia/len(location_array)

                        ### Calculate Centroids of Clusters
                        centroids = centroids_temp
                        #print ("centroids len", len(centroids))
			### Get Labels for all people's locations
                        labels = kmeans.labels_ 
                        tf = time.time()
                        #print centroids
                        print "To META: ", tf-t0
                        metaclustering(centroids)
### Service Handler
def handle_recv(req):
        global tf
        global t0
        print "Clustering Started"
        clustering(req)
        print "Clustering Finished"
        print "Calculating Runtime ..."
        runtime = []
        delay = r_delay(0.05,random.random())
        print "Clustering Finished: ", tf-t0 , " Secs"
        runtime.append([tf-t0, 0])
        runtime.append([delay, 0])
        runtime.append([get_size(PeopleLocsResponse(True)),0])
        time.sleep(delay)
        with open("/DataStorage/clustering_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                writer = csv.writer(f)
                writer.writerows(runtime)
        return PeopleLocsResponse(True)

### Recieve People Poses from UAV Service
def recv_poses():
                #global assigns
                print "Starting the clustering service"
                rospy.Service('/clustering', PeopleLocs, handle_recv)
                print "Successfully finished the clustering service"
                rospy.spin()

### Main Service Client for UAV
def clustering_service():

                ### Recieve People Poses from UAV Service
                recv_poses()


if __name__ == '__main__':
                if rank == 0:
                        rospy.init_node('clustering_service', anonymous=True)
                        clustering_service()
                else:
                        flag = False
                        inertia = 100000
                        location_array = [0]
                        ks = range(1,size+1)
                        data = [1]
                        location_array = comm.bcast(location_array, root=0)

                        ### while the average cluster radius is greater than 10m
                        while((inertia/len(location_array))>= 20 or flag == False):
                                ks_copy = list(ks)
                                data = comm.scatter(ks_copy, root=0)
                                ### Calculate Clusters
                                kmeans = cluster.KMeans(init='k-means++', n_init=10, n_clusters=int(data))
                                kmeans.fit(location_array)
                                inertia = kmeans.inertia_
                                #print "inertia for process ", rank, ": ", inertia
                                min_inertia = comm.reduce(inertia, op=MPI.MIN, root=0)
                                flag = True
                                ks = [x+1 for x in ks]


