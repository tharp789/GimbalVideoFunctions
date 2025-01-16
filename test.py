import siyi_kinematics
import cv2
import os
import sys
import numpy as np
from time import sleep

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
siyi_sdk_path = os.path.join(parent_directory, "siyi_sdk")
  
sys.path.append(parent_directory)
sys.path.append(siyi_sdk_path)

from siyi_sdk.siyi_sdk import SIYISDK

def test_location_accuracy(pitch, yaw, expected_distance):
    siyi_kinematics = siyi_kinematics.SIYIKinematics("camera_calibration.npz")
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    cam.requestSetAngles(yaw, pitch)
    cam.disconnect()
    sleep(1)

    rtsp_url = "rtsp://192.168.144.25:8554/main.264"

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit(1)
    ret, frame = cap.read()
    cap.release()

    pixel = get_pixel_from_click(frame)
    location = siyi_kinematics.get_world_location_from_pixel(pixel[0], pixel[1], 1, pitch, yaw)

    dist_error = np.linalg.norm(location) - expected_distance
    print(f"Location: {location}")
    print(f"Distance error: {dist_error}")

def get_pixel_from_click(image):
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            param.append((x, y))

    cv2.imshow("Click on the desired pixel for 3D location", image)
    points = []
    cv2.setMouseCallback("Click on the desired pixel for 3D location", on_mouse, points)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return points[0]

if __name__ == "__main__":
    pitch = -45
    yaw = 0
    expected_distance = 91
    test_location_accuracy(pitch, yaw, expected_distance)
    


    

