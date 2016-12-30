# Local Transformation functions

from numpy import np
from numpy.linalg import inv

def local_mapping(T_k_0, T_k1):
    T_inv = inv(T_k1)
    return T_k_0 * T_inv


def local_transformation(T_k_0, T_k1):
    T_inv = inv(T_k1)
    return T_k_0 * T_inv

def local_localization(t_n0, t_nmin_0, t_nmin_kmin, t_k_kmin):
    t_nmin_0_inv = inv(t_nmin_0)
    t_k_kmin_inv = inv(t_k_kmin)
    t_n_nmin = t_n0 * t_n_nmin_inv
    return t_n_nmin * t_nmin_kmin * t_k_kmin_inv