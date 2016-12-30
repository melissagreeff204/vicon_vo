
from __future__ import division, print_function
# import rospy
from threading import Lock
import inertial_transformations
import local_transformations
import networkx as nx
import math
import numpy
import tf.transformations as tf
# from quaternions import (omega_from_quat_quat, apply_omega_to_quat,
#                          global_to_body)

# from dsl__estimation_vicon.srv import GetState
# from std_msgs.msg import Empty
# from geometry_msgs.msg import TransformStamped
# from dsl__utilities__msg.msg import StateData
# from dsl__utilities__msg.msg import StateVector

# Needed to send numpy.array as a msg
# from rospy.numpy_msg import numpy_msg

# from dsl__utilities__msg.msg import StateVector
# from dsl__estimation_vicon.srv import GetState
import itertools

class Graph:
   newid = itertools.count().next

   def __init__(self, vertex, edge, maps, repeat, t_0, t):
       self.id = Graph.newid()
       self.vertex = vertex
       self.edge = edge
       self.map = maps
       self.repeat = repeat
       self.inertial_frame = t_0
       self.transformation = t

# Need is to get the state

def checkClosest(map_arr, closest, repeater):

	current = closest[1]
	ret_current = [closest[0], closest[1]]

	try:
		next_map = map_arr[closest[0]+1]
		next_map_2 = map_arr[closest[0]+2]

		ret_next = [closest[0]+1, map_arr[closest[0]+1]]
		ret_next_2 = [closest[0]+2, map_arr[closest[0]+2]]
	except:
		return current
	

	r_repeater = np.array(repeater.inertial_frame[0:2,3])
	r_current = np.array(current.inertial_frame[0:2,3])
	r_next = np.array(next_map.inertial_frame[0:2,3])
	r_next_2 = np.array(next_map_2.inertial_frame[0:2,3])

	dist_current = np.linalg.norm(r_repeater-r_current)
	dist_next = np.linalg.norm(r_repeater-r_next)
	dist_next_2 = np.linalg.norm(r_repeater-r_next_2)

	if dist_current <= dist_next && dist_current <= dist_next_2:
		return closest

	else if dist_next < dist_current && dist_next < dist_next_2:
		return ret_next

	else if dist_next_2 < dist_current && dist_next_2 < dist_next:
		return ret_next_2


if __name__ == '__main__':

	#state 
	# state = self.get_state().state

	# state_vector = np.hstack((state.pos,
							# state.vel,
							# state.euler,
							# state.omega_b))
	map_arr = []
	repeater_arr = []
	# first = True

	# get state
	new_state_map = getState()

	

	# initial map		
	T_0 = T_curr = inertial_transformation(state.pos, state.euler)
	k = 1
	G_first = Graph(k, [k-1,k], True, False, T_curr, T_curr)
	map_arr.append(G_first)



	# get state

	end_mapping = False

	while !end_mapping:
		new_state = getState()
		T_curr = inertial_transformation(new_state.pos, new_state.euler)
		add = check_addition(map_arr[-1].inertial_frame, T_curr)

		if add == "true":
			k = k +1
			T_local = local_mapping(T_curr, map_arr[-1].inertial_frame)
			G_new = Graph(k, [k-1,k], True, False, T_curr, T_local)
			map_arr.append(G_new)


	# repeating phase
	# initial repeater


	local_pose = {}

	closest = [0,map_arr[0]]
	new_state_repeater = getState()
	T_repeater_init = inertial_transformation(state.pos, state.euler)
	k = 1
	G_first_repeater = Graph(k, [k-1,k], True, False, T_repeater_init, T_repeater_init)
	repeater_arr.append(G_first)
	end_repeater = False
	closest_old = closest
	closest = checkClosest(map_arr, closest, G_first_repeater)
	if closest != closest_old:
		if closest[0] == closest_old[0] + 2:
			closest = checkClosest(map_arr, closest, G_first_repeater)

	local_transformation = local_transformation(closest.inertial_frame, G_first_repeater.inertial_frame)
	local_pose[G_first_repeater.vertex] = [closest, G_first_repeater,local_transformation]

	while !end_repeater:
		# closest = [0,map_arr[0]]
		new_state_repeater = getState()
		T_repeater_init = inertial_transformation(state.pos, state.euler)
		k = k + 1
		G_repeater = Graph(k, [k-1,k], True, False, T_repeater_init, T_repeater_init)
		repeater_arr.append(G_first)
		end_repeater = False
		closest_old = closest
		closest = checkClosest(map_arr, closest, G_repeater)
		if closest != closest_old:
			if closest[0] == closest_old[0] + 2:
				closest = checkClosest(map_arr, closest, G_repeater)

		local_transformation = local_transformation(closest.inertial_frame, G_repeater.inertial_frame)
		local_pose[G_repeater.vertex] = [closest, G_repeater,local_transformation]
	
		# this_state = 

	return local_transformation










