# Put initial guess of pose as the previous frame's pose
# Change grad and find covariance
# Parallelize, dont vectorize

# Library imports
import numpy as np
import cv2
import sys
import time
import argparse
import math
from multiprocessing import Pool

# Module imports
from keyframe_utils import *
from pose_estimation.config import *
import pose_estimation.optimiser as optimiser
import pose_estimation.depth_map_fusion as depth_map_fusion
from pose_estimation.stereo_match import *
from params import camera_matrix as cam_matrix,camera_matrix_inv as cam_matrix_inv,sigma_p

def get_initial_pose():
    '''
    Pose for the first frame
    '''
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

# Change later
def get_initial_covariance():
    return np.eye(6)

def calc_photo_residual(i, frame, cur_keyframe, T,step = 0.1):
    '''
    Calculates the photometric residual for one point

    Arguments:
            i: Pixel location (x,y)
            frame: Current frame as numpy array
            cur_keyframe: Previous keyframe as Keyframe object
            T: Estimated pose

    Returns:
            r: Photometric residual
    '''
    # Make i homogeneous
    i = np.append(i, np.ones(1))
    i = i.astype(np.uint16)
    # 3D point 3*1

    #print("i",i)
    V = cur_keyframe.D[i[0]][i[1]] * np.matmul(cam_matrix_inv, i)

    #print("D:",cur_keyframe.D[i[0]][i[1]])
    #print("mat:",np.matmul(cam_matrix_inv, i))
    # Make V homogeneous 4*1
    V = np.append(V, 1)
    #print("V",V)

    # 3D point in the real world shifted (3*4 x 4*1 = 3*1)
    u_prop = np.matmul(T, V)[:3]

    # 3D point in camera frame (3*3 * 3*1)
    u_prop = np.matmul(cam_matrix, u_prop)
    #print("u_prop1",u_prop)
    # Projection onto image plane
    u_prop = (u_prop / u_prop[2])[:2]
    u_prop = u_prop.astype(np.uint64)
    #print("u_prop",u_prop)
    u_prop = fix_u(u_prop)

    r = (cur_keyframe.F[i[0], i[1]] - frame[u_prop[0], u_prop[1]]).astype(np.float64)


    #finding variance
    V = (cur_keyframe.D[i[0]][i[1]]+step)* np.matmul(cam_matrix_inv, i)

    # Make V homogeneous 4*1
    V = np.append(V, 1)

    # 3D point in the real world shifted (3*4 x 4*1 = 3*1)
    u_prop = np.matmul(T, V)[:3]

    # 3D point in camera frame (3*3 * 3*1)
    u_prop = np.matmul(cam_matrix, u_prop)

    # Projection onto image plane
    u_prop = (u_prop / u_prop[2])[:2]
    u_prop = u_prop.astype(np.int64)

    u_prop = fix_u(u_prop)
    #print(i,u_prop)
    U = (cur_keyframe.F[i[0], i[1]] - frame[u_prop[0], u_prop[1]]).astype(np.float64)
    deriv = (U-r)/step

    return r,U


def calc_r_for_delr(u, D, frame, cur_keyframe, T):
    '''
    Finds photometric residual given one point

    Argumemnts:
            u: numpy array oof x and y location
            D: Depth map value at u
            frame: current frame
            cur_keyframe: previous keyframe of keyframe class
            T: current estimated pose

    Returns:
            r: photometric residual
    '''
    u = np.append(u, [1])
    v = D * np.matmul(cam_matrix_inv, u)
    v = np.append(v, [1])
    u_prop = np.matmul(T, v)[:3]
    u_prop = np.matmul(cam_matrix, u_prop)
    u_prop = ((u_prop / u_prop[2])[:2]).astype(int)

    u_prop = fix_u(u_prop)

    r = int(cur_keyframe.F[u[0], u[1]]) - int(frame[u_prop[0], u_prop[1]])
    return r

def delr_delD(u, frame, cur_keyframe, T):
    '''
    Finds the derivative of the photometric residual wrt depth (r wrt d)
    delr/delD  = (delr/delu)*(delu/delD)
    delr/delu = delr/delx + delr/dely - finding root of sum of squares now
    delD/delu = delD/delx + delD/dely - finding root of sum of squares now
    r = cur_keyframe.I[u[0]][u[1]] - frame[u_prop[0]][u_prop[1]] - How r is defined normally

    Arguments:
            u: High gradient pixel location
            frame: Current frame as numpy array
            cur_keyframe: Previous keyframe as a Keyframe object
            T: Estimated pose

    Returns:
            delr: The derivative
    '''

    # Convert u to int
    u = u.astype(int)

    # For finding right and left sides for x and y
    ulx = np.array([u[0] - 1, u[1]])
    urx = np.array([u[0] + 1, u[1]])
    uly = np.array([u[0], u[1] - 1])
    ury = np.array([u[0], u[1] - 1])

    ulx = fix_u(ulx)
    uly = fix_u(uly)
    urx = fix_u(urx)
    ury = fix_u(ury)

    # Depth map values
    Dlx = cur_keyframe.D[ulx[0]][ulx[1]]
    Drx = cur_keyframe.D[urx[0]][urx[1]]
    Dly = cur_keyframe.D[uly[0]][uly[1]]
    Dry = cur_keyframe.D[ury[0]][ury[1]]

    # Finding delD/delu
    delDdelu = ((Drx - Dlx)**2 + (Dry - Dly)**2)**0.5
    deludelD = 1.0 / delDdelu

    r_list = [0, 0, 0, 0]  # Just random

	# u = np.append(u,[1])
	# v = D*np.matmul(cam_matrix_inv,u)
	# v = np.append(v,[1])
	# u_prop = np.matmul(T,v)[:3]
	# u_prop = np.matmul(cam_matrix,u_prop)
	# u_prop = ((u_prop/u_prop[2])[:2]).astype(int)
	# r_list[0] = cur_keyframe.I[u[0],u[1]] - frame[u_prop[0],u_prop[1]]

    # Calculate r_list
    calc_r_for_delr_v = np.vectorize(
        calc_r_for_delr, excluded=[
            2, 3, 4], signature='(1),()->()')
    u_list = [ulx, urx, uly, ury]
    D_list = [Dlx, Drx, Dly, Dry]
    r_list = calc_r_for_delr_v(u_list, D_list, frame, cur_keyframe, T)

    delrdelu = ((r_list[0] - r_list[1])**2 + (r_list[2] - r_list[3])**2)**0.5

    delr = delrdelu * deludelD
    return delr

def calc_photo_residual_uncertainty(u, frame, cur_keyframe, T):
    '''
    Calculates the photometric residual uncertainty

    Arguments:
            u: High gradient pixel location
            frame: Current frame as a numpy array
            cur_keyframe: Previous keyframe as a Keyframe object
            T: Estimated pose

    Returns:
            sigma: Residual uncertainty
    '''
    deriv = delr_delD(u, frame, cur_keyframe, T)
    sigma = (sigma_p**2 + (deriv**2) * cur_keyframe.U[u[0]][u[1]])**0.5
    return sigma

def ratio_residual_uncertainty(u, frame, cur_keyframe, T):
    r,deriv = calc_photo_residual(u, frame, cur_keyframe, T)
    sigma = (sigma_p**2 + (deriv**2) * cur_keyframe.U[u[0]][u[1]])**0.5
    #print(r)
    return huber_norm(r/sigma)
    #return huber_norm(calc_photo_residual(u, frame, cur_keyframe, T) /calc_photo_residual_uncertainty(u, frame, cur_keyframe, T))

def calc_cost_parallel(uu,frame,cur_keyframe,T):
    pool = Pool(processes = 2)
    processes = []
    for i in uu:
        processes.append([i,frame,cur_keyframe,T])
    out = pool.starmap(ratio_residual_uncertainty,processes)
    return out

# WHAT IS FLAG?
def calc_cost(uu, frame, cur_keyframe, T,flag = 0):
    '''
    Calculates the residual error as a stack.

    Arguments:
            uu: An array containing the high gradient elements (X,2)
            frame: Numpy array o the current frame
            cur_keyframe: Previous keyframe as a Keyframe class
            pose: Current estimated Pose
            flag = 1 if you wanna do in parallel

    Returns:
            r: Residual error as an array
    '''
    if flag == 1:
        return calc_cost_parallel(uu,frame,cur_keyframe,T)
    print("No\n\n")
    return ratio_residual_uncertainty_v(uu, frame, cur_keyframe, T)



def loss_fn_for_jack(uu, frame, cur_keyframe, T_s):
    T = get_back_T(T_s)
    cost = calc_cost(uu, frame, cur_keyframe, T,1)
    return np.array(cost) 

def loss_fn(uu, frame, cur_keyframe, T_s):

    T = get_back_T(T_s)
    cost = calc_cost(uu, frame, cur_keyframe, T,1)
    cost = np.sum(cost)
    cost = cost/len(uu)
    return cost 

def get_W(dof, stack_r):
    '''
    Returns the weight matrix for weighted Gauss-Newton Optimization

    Arguments:
            dof: Number of high gradient elements we are using
            stack_r: The stacked residual error as a numpy array (of length dof)

    Returns:
            W: Weight Matrix
    '''
    W = np.zeros((dof, dof))
    for i in range(dof):
        W[i][i] = (dof + 1) / (dof + stack_r[i]**2)
    return W

def find_covariance_matrix(u, frame, cur_keyframe, T_s,frac = 0.01):
    '''
    Find 6x6 covariance matrix by inverse of hessian
    '''
    J = np.zeros((len(u),6))
    fract = np.zeros(6)
    for i in range(6):
        fract[i] = frac
        costa = loss_fn_for_jack(u, frame, cur_keyframe, T_s * (1 - fract)) # Stacked residual error
        costb = loss_fn_for_jack(u, frame, cur_keyframe, T_s * (1 + fract)) 
        J[:,i] = (costb - costa) / (2*T_s[i]*frac)
    # We have J(dofx6)
    W = get_W(len(u),loss_fn_for_jack(u,frame,cur_keyframe,T_s)) # dof x dof
    H = np.matmul(np.matmul(J.T,W),J) # 6x6
    if np.linalg.det(H)==0:
        print("Zero det encountered\n\n")
        return np.linalg.pinv(H)
        # Put error handling
    C = np.linalg.inv(H)
    return C # Estimate of covariance

def grad_fn(u, frame, cur_keyframe, T_s, frac = 0.01):
    '''
    Calculate gradients
    '''
    grad = np.zeros(6)
    fract = np.zeros(6)
    """
    for i in range(6):
        fract[i] = frac
        costa = loss_fn(u, frame, cur_keyframe, T_s * (1 - fract))
        costb = loss_fn(u, frame, cur_keyframe, T_s * (1 + fract))
        grad[i] = (costb - costa) / (2*T_s[i]*frac)
        fract[i] = 0
    """
    costa = loss_fn(u, frame, cur_keyframe, T_s * (1 - frac))
    costb = loss_fn(u, frame, cur_keyframe, T_s * (1 + frac))
    grad = (costb - costa)/(2*T_s*frac)
    return grad # 6x1 gradient

# Vectorised implementations
ratio_residual_uncertainty_v = np.vectorize(
        ratio_residual_uncertainty, excluded=[
            1, 2, 3], signature='(1)->()')


def minimize_cost_func(u, frame, cur_keyframe, 
    variance = 0.01,
    mean = 5.0,
    learning_rate = 0.3,
    max_iter= 2,
    loss_bound = 0.5):


    frame = frame.astype(np.float64)
    cur_keyframe.F.astype(np.float64)

    dof = len(u)
    T_s = np.random.random((6)) * variance

    """
    T = get_back_T(T_s)
    u = [85,306]
    r,c = calc_photo_residual(u,frame,cur_keyframe,T)
    print("\n\nyolo",r)
    print(c)
    return 1
    """
    
    optim = optimiser.SGD(lr = learning_rate)
    i = 0

    while True:  # Change later

        loss = loss_fn(u, frame, cur_keyframe, T_s)
        print("loss old:", loss)
        grads = grad_fn(u, frame, cur_keyframe, T_s)
        print("grads:",grads)
        print("T_S old:",T_s)
        T_s = optim.get_update([T_s],[grads])[0]
        print("T_S new:",T_s)
        print("loss new:",loss_fn(u,frame,cur_keyframe,T_s))
        i = i + 1
        # Stopping condtions
        if (loss < loss_bound) or (i == max_iter):# or (abs(np.max(grads)) > 10000):
            break

    print(loss_fn(u, frame, cur_keyframe, T_s))
    C = find_covariance_matrix(u,frame,cur_keyframe,T_s)
    return get_back_T(T_s), C,loss_fn(u, frame, cur_keyframe, T_s)

def test_min_cost_func():
    '''
    Test minimum cost function:

    * Take current frame, keyframe
    '''
    # Image Size
    im_x, im_y = im_size

    # Random high grad points
    u_test = np.array([[5, 4], [34, 56], [231, 67], [100, 100], [340, 237]])

    # Random frame
    frame_test = np.uint8(np.random.random((im_x, im_y)) * 256)

    # Current key frame, depth, pose, uncertainuty
    cur_key_test_im = np.uint8(np.random.random((im_x, im_y, 3)) * 256)
    cur_key_test_im_grey = np.uint8(
        (cur_key_test_im[:, :, 0] + cur_key_test_im[:, :, 1] + cur_key_test_im[:, :, 2]) / 3)
    cur_key_depth = np.random.random((im_x, im_y))
    dummy_pose = np.eye(4)[:3]
    cur_key_unc = np.ones((im_x, im_y))

    cur_key = Keyframe(
        dummy_pose,
        cur_key_depth,
        cur_key_unc,
        cur_key_test_im_grey)

    print(
        "Testing minimize cost func",
        minimize_cost_func(
            u_test,
            frame_test,
            cur_key))

if __name__ == '__main__':
    test_min_cost_func()
