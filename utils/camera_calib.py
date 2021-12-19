import numpy as np
import cv2
import glob
import argparse

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


n_rows = 9
n_cols = 9

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((n_rows * n_cols,3), np.float32)
objp[:,:2] = np.mgrid[0:n_cols,0:n_rows].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
imgindex = [] # filenames with detectable boards

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--input', help='Path to input image.', default='./calib_images')
args = parser.parse_args()

images = glob.glob(args.input + '/*.png')

count = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (n_rows , n_cols),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
        imgindex.append(fname)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (n_rows , n_cols), corners,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
        count += 1

print("Num img: " + str(count))

cv2.destroyAllWindows()

print("Returned images: ", imgindex)
fname = imgindex[0]

img = cv2.imread(fname)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

print("Return: ", ret)
print("Intrinsic Camera Matrix: ", mtx)
print("Distortion: ", dist)
print("Rotation: ", rvecs)
print("Translation: ", tvecs)

# rotation_matrix = np.zeros(shape=(3,3))
rvecs_l = np.array(rvecs)
rotation_matrix, eb = cv2.Rodrigues(rvecs_l[0])
print(rotation_matrix)


rotation_matrix, eb = cv2.Rodrigues(rvecs_l[1])
print(rotation_matrix)

rotation_matrix, eb = cv2.Rodrigues(rvecs_l[2])
print(rotation_matrix)

rotation_matrix, eb = cv2.Rodrigues(rvecs_l[3])
print(rotation_matrix)
