import cv2
import numpy as np
import glob

# Parameters
CHECKERBOARD = (10, 7)  # Number of inner corners in the checkerboard
square_size = 0.025  # Size of a square in your checkerboard pattern (e.g., in meters or millimeters)
image_folder = "Images"  # Replace with the path to your image folder

# Termination criteria for corner subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for the checkerboard pattern
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load images
images = glob.glob(f"{image_folder}/*.jpg")  # Adjust file extension if necessary

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, square_size)

    if ret:
        objpoints.append(objp)

        # Refine the corners
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Checkerboard', img)
        cv2.waitKey()
    else:
        print(f"Error: Could not find corners in {fname}")
        cv2.waitKey()

cv2.destroyAllWindows()

# Camera calibration
ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, distortion)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print(f"Total Reporjection error: {mean_error / len(objpoints)}")

if ret:
    print("Camera matrix:")
    print(camera_matrix)
    print("\ndistortionortion coefficients:")
    print(distortion)

    # Save the calibration results
    np.savez("camera_calibration.npz", camera_matrix=camera_matrix, distortion=distortion)
    print("Calibration data saved to 'camera_calibration.npz'.")
else:
    print("Camera calibration failed.")
