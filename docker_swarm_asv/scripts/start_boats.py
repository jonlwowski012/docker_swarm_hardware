#!/usr/bin/env python
import subprocess
import sys
import os
sys.path.append('/DataStorage')
import config

number_boats = config.params['asvs']
for i in range(number_boats):
	#subprocess.Popen(['/bin/bash', '-c', 'export', 'ROS_NAMESPACE=/asv'+str(i)])
	subprocess.Popen(['/bin/bash', '-c', 'source /root/.bashrc;', 'export ROS_NAMESPACE=/asv;', 'rosrun docker_swarm asv_service.py'])



