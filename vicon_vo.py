
from __future__ import division, print_function
from threading import Lock
import inertial_transformations
import local_transformations
import networkx as nx
import math
import numpy as np
import tf.transformations as tf
import itertools

class Graph:
    # generate new id for node using itertools
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
    # check closest function, passing in array of all map nodes, current repeater
    # node and the last calculated closest map node in the graph
	current = closest[1]
	ret_current = [closest[0], closest[1]]

    # try to get the next two map nodes to see which is closest
    # if the last item in the map array is the current closest node and if that
    # node is the closest node currently, then return that node
	try:
		next_map = map_arr[closest[0]+1]
		next_map_2 = map_arr[closest[0]+2]
		ret_next = [closest[0]+1, map_arr[closest[0]+1]]
		ret_next_2 = [closest[0]+2, map_arr[closest[0]+2]]
	except:
		return current

    # get the r for the repeater, current closest node, and the two subsequent nodes
	r_repeater = np.array(repeater.inertial_frame[0:2,3])
	r_current = np.array(current.inertial_frame[0:2,3])
	r_next = np.array(next_map.inertial_frame[0:2,3])
	r_next_2 = np.array(next_map_2.inertial_frame[0:2,3])

    # get the distances for the current closest node and the two subsequent nodes
	dist_current = np.linalg.norm(r_repeater-r_current)
	dist_next = np.linalg.norm(r_repeater-r_next)
	dist_next_2 = np.linalg.norm(r_repeater-r_next_2)

    # measure the distances to determine which is the closest to the current repeater node in the graph
    # return this node
	if dist_current <= dist_next && dist_current <= dist_next_2:
		return closest

	else if dist_next < dist_current && dist_next < dist_next_2:
		return ret_next

	else:
		return ret_next_2


if __name__ == '__main__':
    # initialize map array and repeater array for map and repeater nodes to be added to
	map_arr = []
	repeater_arr = []

    # get the first state in the map
	new_state_map = getState()

	# initial map
    # get the transformation matrices for the first state in the map
    # add that node to the graph
	T_0 = T_curr = inertial_transformation(state.pos, state.euler)
	k = 1
	G_first = Graph(k, [k-1,k], True, False, T_curr, T_curr)
	map_arr.append(G_first)

    # set filler variable to False. This will be used to test when the program should end the mapping phase
	end_mapping = False

	while !end_mapping:
        # while mapping phase is note completed
        # get current state and inertial transformation matrix

		new_state = getState()
		T_curr = inertial_transformation(new_state.pos, new_state.euler)
		add = check_addition(map_arr[-1].inertial_frame, T_curr)

        # check if we need to add a node to the map array
		if add == "true":
            # add node to map array with increasing vertex number
			k = k +1
			T_local = local_mapping(T_curr, map_arr[-1].inertial_frame)
			G_new = Graph(k, [k-1,k], True, False, T_curr, T_local)
			map_arr.append(G_new)

    # initialize local_pose variable. This will be used to store the closest inertial frame,
    # the repeater node and the local transformation that has been calculated
    # this is a dictionary which maps to the repeater local transformation
	local_pose = {}


    # set closest as the first node in the map by default
    # get current state to initialize repeater
	closest = [0,map_arr[0]]
	new_state_repeater = getState()

	T_repeater_init = inertial_transformation(state.pos, state.euler)
	k = 1
	G_first_repeater = Graph(k, [k-1,k], True, False, T_repeater_init, T_repeater_init)
	repeater_arr.append(G_first)
	closest_old = closest

    # find the closest node the the current repeater node, passing in the current closest node
	closest = checkClosest(map_arr, closest, G_first_repeater)

    # check if the new closest node is equal to the old closest node
	if closest != closest_old:
        # if the new closest node is equal to the node two positions in front of the old closest,
        # run the function again to make sure that this is in fact the closest node
		if closest[0] == closest_old[0] + 2:
			closest = checkClosest(map_arr, closest, G_first_repeater)

    # get the local transformation from the current repeater node to the current transformation node
	local_trans = local_transformation(closest.inertial_frame, G_first_repeater.inertial_frame)
	local_pose[G_first_repeater.vertex] = [closest, G_first_repeater,local_trans]

    end_repeater = False
	while !end_repeater:
		# set closest as the previous closest in the map by default
        # get current state to initialize repeater
		new_state_repeater = getState()
		T_repeater_init = inertial_transformation(state.pos, state.euler)
		k = k + 1
		G_repeater = Graph(k, [k-1,k], True, False, T_repeater_init, T_repeater_init)
		repeater_arr.append(G_first)
		end_repeater = False
		closest_old = closest

        # find the closest node the the current repeater node, passing in the current closest node
		closest = checkClosest(map_arr, closest, G_repeater)

        # check if the new closest node is equal to the old closest node
		if closest != closest_old:
            # if the new closest node is equal to the node two positions in front of the old closest,
            # run the function again to make sure that this is in fact the closest node
			if closest[0] == closest_old[0] + 2:
				closest = checkClosest(map_arr, closest, G_repeater)

        # get the local transformation from the current repeater node to the current transformation node
		local_trans = local_transformation(closest.inertial_frame, G_repeater.inertial_frame)
		local_pose[G_repeater.vertex] = [closest, G_repeater,local_trans]


    # return the local transformations for each of the repeater nodes for matlab processing

	return local_trans
