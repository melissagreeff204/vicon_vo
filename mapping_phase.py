import networkx as nx
import math
import numpy
import tf.transformations as tf

def mapping_phase(G, I, T_curr, k) 

''' 
Parameters:
Graphs G (graph with local transformations stored) and graphs I (graph with inertial transformation stored), the current pose (T_curr) and the current potential vertex to be added k
Outputs:
Updated graphs G and I
'''

#extracct T_kprev_0 from graph I

# if rotation or distance exceeds ... then add to graph
   check_addition(T_kprev_0 ,T_curr)

   if add == "true" #update graphs accordingly

	#adding a new node to the graph: graph like in VTR
		G.add_node(k)
		G.add_edge(k-1,k, Transformation = T_k_0)

	#inertial graph to store transformation in the intertial frame:	
		I.add_node(k)
		I.add_edge(k-1,k, Transformation = T_k_0)

	return G, I		#return updated graph objects

