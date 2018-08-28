. /DataStorage/config.config
echo "$uavs"
for i in `seq 1 $asvs`;
do
	export ROS_NAMESPACE=/asv$i
	rosrun docker_swarm asv_service.py &
done
#export ROS_NAMESPACE=/asv1
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv2
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv3
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv4
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv5
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv6
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv7
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv8
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv9
#rosrun docker_swarm asv_service.py &
#export ROS_NAMESPACE=/asv10
#rosrun docker_swarm asv_service.py &

