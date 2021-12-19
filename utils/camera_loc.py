import numpy as np
import cv2
import argparse
import yaml

def calculate_loc_pixel(s, K, P, X):
    # dimension checks
    # import pdb; pdb.set_trace()

    R = P[0:3,0:3]
    t = P[0:3,3].reshape((3,1))
    u = 1/s*np.matmul(K, R.dot(X)+t) # np.matmul
    return u

def calculate_loc_cartesian(s, K, P, U):
    # X = np.transpose(np.linalg.pinv(P))[0:3,:]*np.linalg.inv(K)*s*U
    # Incorrect?
    R = P[0:3,0:3]
    t = P[0:3,3].reshape((3,1))
    K_i = np.linalg.inv(K)
    R_i = np.linalg.inv(R)
    dis = (s*K_i.dot(U)-t)
    X = R_i.dot(dis)
    return X

def calculate_stereo_loc_cartesian(cx, px, cy, py, fx, fy, B, D):
    z = f*B/D
    x = z*(px-cx)/fx
    y = z*(py-cy)/fy
    return np.asarray([x, y, z]).reshape((3,1))

def calculate_stereo_loc_cartesian_given_depth(cx, px, cy, py, f, depth):
    x = depth*(px-cx)/f
    y = depth*(py-cy)/f
    return np.asarray([x, y, depth]).reshape((3,1))

# from matplotlib import pyplot as plt

# Calibration ...
# 618.0386719675123
# P = np.asarray([147.80, 0.0, 256.0, 0.0, 0.0, 147.80, 256.0, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape((3,4))
# K = np.asarray([[147.80, 0.0, 256.0, 0.0, 147.80, 256.0, 0.0, 0.0, 1.0]]).reshape((3,3))
# P = np.asarray([1.16392310e+03, 0.0, 6.28041958e+02, 0.0, 0.0, 1.12263977e+03, 3.42072773e+02, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape((3,4))
# K = np.asarray([[1.16392310e+03, 0.0, 6.28041958e+02, 0.0, 1.12263977e+03, 3.42072773e+02, 0.0, 0.0, 1.0]]).reshape((3,3))
K = np.asarray([[670.72030436,   0.,         265.020138  ], [  0.,         670.75778801, 245.91319719], [  0.,           0.,           1.        ]])
Rot = np.asarray([[ 9.99999998e-01,  3.66661792e-05,  5.16035318e-05], [-3.69199722e-05,  9.99987863e-01,  4.92675549e-03], [-5.14222602e-05, -4.92675738e-03,  9.99987862e-01]])
trn = np.array([[-4.29449865], [-2.27970549], [20.77305481]])
P = np.hstack([Rot, trn])
s = 1
U = np.asarray([128, 128, 1]).reshape((3,1))

# import pdb; pdb.set_trace()
print("Camera to world")
print(U, "\n->\n", calculate_loc_cartesian(s, K, P, U))

# print("World to camera")
# X = np.asarray([-0.41488377, -0.41488377, 1]).reshape((3,1))
# print(X, "\n->\n", calculate_loc_pixel(s, K, P, U))
