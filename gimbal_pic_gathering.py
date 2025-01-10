from time import sleep
import sys
import os
import cv2
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
siyi_sdk_path = os.path.join(parent_directory, "siyi_sdk")
  
sys.path.append(parent_directory)
sys.path.append(siyi_sdk_path)

from siyi_sdk.siyi_sdk import SIYISDK
from siyi_sdk.stream import SIYIRTSP


def get_images(num_images):
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)
        
    cam.requestSetAngles(0, 45)
    cam_str = cam.getCameraTypeString()
    cam.disconnect()
    sleep(1)

    rtsp_url = "rtsp://192.168.144.25:8554/main.264"

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit(1)

    existing_index = len([name for name in os.listdir("Images") if os.path.isfile(os.path.join("Images", name))])

    for i in range(num_images):
        print(f"press Enter to get image {i}")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                exit(1)
            
            # Display the captured frame
            cv2.imshow('Camera', frame)
            if cv2.waitKey(1) == 13:  # Enter key
                break
        
        # Save or display the captured image
        image_filename = f"Images/image_{i + existing_index}.jpg"
        cv2.imwrite(image_filename, frame)  # Save the frame as an image
        print(f"Captured and saved {image_filename}.")
        
    cap.release()

    print("Got the Images")
if __name__ == "__main__":
    get_images(1)