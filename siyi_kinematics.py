import numpy as np
import cv2

SIYI_VERTICAL_OFFSET = 0.06 #meters
SIYI_HORIZONTAL_OFFSET = 0.01 #meters

# Frame convetion is z forward, x right, y down

class SIYIKinematics:
    def __init__(self, camera_calibration_file_path):
        camera_calibration = np.load(camera_calibration_file_path)
        self.calibration_matrix = camera_calibration['camera_matrix']
        self.distortion = camera_calibration['distortion']

    def get_camera_transform(self, yaw, pitch):
        '''
        Get the camera transform from the base to the camera, from on the gimbal angles. 
        Coordinate frame for the camera is z forward, x right, y down.
        Coordinate frame for the vehicle is z forward, x right, y down.

        Parameters:
        yaw (float): The yaw angle of the gimbal
        pitch (float): The pitch angle of the gimbal
        base_location (numpy.ndarray): The location of the base in the world frame (3,) with x,y,z

        Returns:
        numpy.ndarray: The transform from the base to the camera (4,4)
        '''

        # Get the rotation matrix for the gimbal
        cam_transformation = np.eye(4)
        # yaw rotation is around the y axis
        yaw_rot = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
        # pitch rotation is around the x axis
        pitch_rot = np.array([[1, 0, 0], [0, np.cos(pitch), -np.sin(pitch)], [0, np.sin(pitch), np.cos(pitch)]])

        # rotation order is first * second * third ....
        cam_rot = np.dot(yaw_rot, pitch_rot)
        x_offset = SIYI_HORIZONTAL_OFFSET * np.sin(yaw)
        z_offset = SIYI_HORIZONTAL_OFFSET * np.cos(yaw)
        y_offset = SIYI_VERTICAL_OFFSET

        cam_transformation[:3,:3] = cam_rot
        cam_transformation[:3,3] = [x_offset, y_offset, z_offset]

        return cam_transformation

    def get_3d_ray_from_pixel(self, pixel_x, pixel_y, camera_matrix, distortion):
        '''
        Get the 3D ray of a pixel in the camera frame. 

        Parameters:
        pixel_x (int): The x pixel location of the object
        pixel_y (int): The y pixel location of the object
        depth (float): The depth of the object
        camera_matrix (numpy.ndarray): The camera matrix (3,3)
        camera_transform (numpy.ndarray): The transform from the base to the camera (4,4)

        Returns:
        numpy.ndarray: The 3D ray of the object in the camera frame (3,)
        '''

        # gets ideal points in camera frame
        undistorted_point = cv2.undistortPoints(np.array([[pixel_x, pixel_y]]).astype(np.float32), camera_matrix, distortion).squeeze()
        x, y = undistorted_point
        ray = np.array([x, y, 1.0])
        ray = ray / np.linalg.norm(ray)
        return ray

    def get_world_location_from_pixel(self, pixel_x, pixel_y, altitude, pitch, yaw):
        '''
        Get the 3D location of a pixel in the world frame. 

        Parameters:
        pixel_x (int): The x pixel location of the object
        pixel_y (int): The y pixel location of the object
        depth (float): The depth of the object
        camera_matrix (numpy.ndarray): The camera matrix (3,3)
        camera_transform (numpy.ndarray): The transform from the base to the camera (4,4)

        Returns:
        numpy.ndarray: The 3D location of the object in the world frame (3,)
        '''

        cam_ray = self.get_3d_ray_from_pixel(pixel_x, pixel_y, self.calibration_matrix, self.distortion)
        base_to_cam = self.get_camera_transform(yaw, pitch)
        cam_to_base = np.linalg.inv(base_to_cam)
        base_ray = np.dot(cam_to_base[:3,:3], cam_ray)
        base_ray = base_ray / np.linalg.norm(base_ray)
        ray_distance = altitude / base_ray[1]
        location = - base_ray * ray_distance #negative to signify the object is below the camera
        return location
    
def test_math(img_path, camera_calibration_file_path, altitude, pitch, yaw, pixel_x=0, pixel_y=0):
    img = cv2.imread(img_path)
    if pixel_x == 0 and pixel_y == 0:
        pixel_x = img.shape[1] // 2
        pixel_y = img.shape[0] // 2
    cv2.circle(img, (pixel_x, pixel_y), 10, (0, 255, 0), 2)
    cv2.imshow("Image", img)
    cv2.waitKey()

    siyi_kinematics = SIYIKinematics(camera_calibration_file_path)
    ray = siyi_kinematics.get_world_location_from_pixel(pixel_x, pixel_y, altitude, pitch, yaw)
    print(ray)

if __name__ == "__main__":
    pitch = -45
    yaw = 0
    altitude = 91
    test_math("Images/image_12.jpg", "camera_calibration.npz", altitude=altitude, pitch=pitch, yaw=yaw, pixel_x=1000, pixel_y=1920//2)
        









