import math
import numpy
import tf.transformations as tf

def inertial_transformation(pos, euler):

	'''
	Parameters:
	----------------
	Take in linear position and euler angles from the state and 
	Output:
	--------------
	Output the transformation in the inertial frame

	'''
	R = eulerAnglesToRotationMatrix(euler) #enter euler angles from vicon and extract rotation matrix

	T_curr[0:2,0:2] = R
	T_curr[0:2,3] = pos

	T_curr[3,0:2]= [0.0, 0.0, 0.0]
	T_curr[3,3]= 1.0

	return T_curr

# checks whether to add vertice to map or not
def check_addition(T_kprev_0 ,T_curr)

'''
	Parameters: 
	---------------
	current pose (T_curr) and the pose of the previous vertex stored in the inertial I graph (T_kprev_0)
	Output:
	---------------
	output whether there was been a sufficient distance or rotation in the current pose to insert a new vertex in the map (i.e. "add=true")

'''

	#linear positions to find linear distance 
	r_k_0 = np.array(T_k_0[0:2,3])
	r_kprev_0 = np.array(T_kprev_0[0:2,3])

	dist = np.linalg.norm(r_k_0-r_kprev_0)

	#extract euler angles - find rotation - angle is maximum
	



	if dist > 0.2 #greater than 20cm
		add = "true"
	else if angle_max > 0.087 #angle difference of greater than approx 5 degrees
		add = "true"

	return add = "true"	

 # Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,  0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])
                        
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]) ],
                    [0, 1, 0 ],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])
                 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R	