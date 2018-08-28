#!/usr/bin/env python

# Title: Clustering service
# Description: Recieves cluster locations and metaclusters them
# Engineer: Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
# Lab: Autonomous Controls Lab, The University of Texas at San Antonio


#########          Libraries         ###################
import time
import sys
import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#from docker_swarm.msg import *
from docker_swarm.srv import *
from tsp_solver.greedy import solve_tsp
from collections import defaultdict
import smtplib  
from email.mime.text import MIMEText
import csv


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

### Dijkstra Class 
class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance

def distance(p0, p1):
        return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

### Setup TSP Solver
def setup_tsp(clusters):
        people_location_array = clusters.poses
        graph_people_locations = Graph()
        number = 1
        people_location_array_current = people_location_array
        #print people_location_array_current
        w, h = len(people_location_array_current)+1, len(people_location_array_current)+1
        cost_mat = [[0 for x in range(w)] for y in range(h)] 
        graph_people_locations.add_node(str(0))
        for i in range(len(people_location_array_current)):
                graph_people_locations.add_node(str(number))
                number = number + 1
        for node in graph_people_locations.nodes:
                for node2 in graph_people_locations.nodes:
                        if(node != '0' and node2 != '0'):
                                cost = distance((people_location_array_current[int(node)-1].position.x, people_location_array_current[in$
                                        (people_location_array_current[int(node2)-1].position.x,
                                        people_location_array_current[int(node2)-1].position.y))
                        else:
                                cost = 9999999
                        graph_people_locations.add_edge(node,node2,cost)
                        cost_mat[int(node)][int(node2)]=cost
        return cost_mat


### TSP Solver
def tsp_solver(location,clusters):
        cost_mat = setup_tsp(clusters)
        path = solve_tsp( cost_mat )

        path_array = PoseArray()
        for index in path:
                path_array.poses.append(clusters.poses[index-1])
        return path_array


### Service Handler
def handle_recv(req):
        t0 = time.time()
        path = tsp_solver(req.location, req.clusters)
        runtime = []
        runtime.append([time.time()-t0, 0])
        delay = r_delay(0.05,random.random())
        runtime.append([delay, 0])
        runtime.append([get_size(TSPResponse(path))*len(path.poses),0])
        time.sleep(delay)
        with open("/DataStorage/tsp_runtime" + rospy.get_namespace().replace("/", '')  + ".csv", "wb") as f:
                writer = csv.writer(f)
                writer.writerows(runtime)

        '''runtime = time.time() - t0
        print "Runtime: ", runtime
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login("jonathan.lwowski@gmail.com", "doctoralrobot_ticsstudent")
        msg = MIMEText("TSP Runtime: " + str(runtime))
        server.sendmail("jonathan.lwowski@gmail.com", "jonathan.lwowski@gmail.com", msg.as_string())

        server.quit()'''
        return TSPResponse(path)

### Receive People Poses from UAV Service
def recv_info():
        print "Starting the traveling salesman service"
        rospy.Service('/traveling_salesman_service', TSP, handle_recv)
        print "Successfully finished the traveling salesman service"
        rospy.spin()



### Main Service Client for TSP
def tsp_service():

        ### Receive location, clusters and metaclusters from ASV
        recv_info()


if __name__ == '__main__':
        rospy.init_node('tsp_service', anonymous=True)
        tsp_service()

