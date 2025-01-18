import numpy as np
import cv2

SIYI_VERTICAL_OFFSET = 0.055 #meters
SIYI_HORIZONTAL_OFFSET = 0.010 #meters

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

        # rotation order is first * second * third ... for local rotations
        cam_rot = np.dot(yaw_rot, pitch_rot)
        x_offset = SIYI_HORIZONTAL_OFFSET * np.sin(yaw)
        z_offset = SIYI_HORIZONTAL_OFFSET * np.cos(yaw)
        y_offset = SIYI_VERTICAL_OFFSET

        cam_transformation[:3,:3] = cam_rot
        cam_transformation[:3,3] = [x_offset, y_offset, z_offset]

        return cam_transformation

    def get_3d_ray_from_pixel(self, pixel_u, pixel_v, camera_matrix, distortion):
        '''
        Get the 3D ray of a pixel in the camera frame. 

        Parameters:
        pixel_u (int): The x pixel location of the object
        pixel_v (int): The y pixel location of the object
        camera_matrix (numpy.ndarray): The camera matrix (3,3)
        distortion (numpy.ndarray): The distortion coefficients (5,)

        Returns:
        numpy.ndarray: The 3D ray of the object in the camera frame (3,)
        '''

        # gets ideal points in camera frame from pixel location
        undistorted_point = cv2.undistortPoints(np.array([[pixel_u, pixel_v]]).astype(np.float32), camera_matrix, distortion).squeeze()
        x, y = undistorted_point
        ray = np.array([x, y, 1.0])
        ray = ray / np.linalg.norm(ray)
        return ray
    
    def undistort_image(self, image):
        '''
        Undistort an image using the camera calibration matrix and distortion coefficients

        Parameters:
        image (numpy.ndarray): The image to undistort

        Returns:
        numpy.ndarray: The undistorted image
        '''
        return cv2.undistort(image, self.calibration_matrix, self.distortion)

    def get_location_base_from_pixel(self, pixel_u, pixel_v, altitude, pitch, yaw):
        '''
        Get the 3D location of a pixel in the gimbal base frame 

        Parameters:
        pixel_u (int): The x pixel location of the object
        pixel_v (int): The y pixel location of the object
        depth (float): The depth of the object
        camera_matrix (numpy.ndarray): The camera matrix (3,3)
        camera_transform (numpy.ndarray): The transform from the base to the camera (4,4)

        Returns:
        numpy.ndarray: The 3D location of the object in the world frame (3,)
        '''

        yaw_rad = - np.radians(yaw) # yaw is in the opposite direction as the coordinate frame
        pitch_rad = np.radians(pitch) # pitch is in the same direction as the coordinate frame
        base_to_cam = self.get_camera_transform(yaw_rad, pitch_rad)

        # get the 3D location of the object in the camera frame
        cam_ray = self.get_3d_ray_from_pixel(pixel_u, pixel_v, self.calibration_matrix, self.distortion)
        # camera pitch is global (with gravity), yaw is local to camera initialization which is aligned with drone heading

        # frame transform from base to cam, is the same as transform from points in cam to points in base (or vectors too)
        # reference: http://www.cs.cmu.edu/afs/cs/academic/class/15494-s24/lectures/kinematics/jennifer-kay-kinematics-2020.pdf
        # reference: https://www.dgp.toronto.edu/~neff/teaching/418/transformations/transformation.html
        ray_in_base_frame_shot_from_cam = np.dot(base_to_cam[:3,:3], cam_ray)
        camera_altitude = altitude - SIYI_VERTICAL_OFFSET
        ray_scale = camera_altitude / ray_in_base_frame_shot_from_cam[1]
        location_cam = ray_in_base_frame_shot_from_cam * ray_scale
        location_base = location_cam + base_to_cam[:3,3]
        return location_base
    
    def get_xy_bounding_box(self, pixel_u, pixel_v, altitude, pitch, yaw, pixel_bounding=1):
        '''
        Get the real world metric bounding box around where the target location might be

        Parameters:
        pixel_u (int): The x pixel location of the object
        pixel_v (int): The y pixel location of the object
        altitude (float): The altitude of the camera
        pitch (float): The pitch angle of the gimbal
        yaw (float): The yaw angle of the gimbal

        Returns:
        dict: The bounding box in the x and y directions

        Notes:
        Since there are more pixels on the vertical axis than the horizontal axis, the bounding box is larger in the vertical direction
        '''
        location = self.get_location_base_from_pixel(pixel_u, pixel_v, altitude, pitch, yaw)
        surrounding_locations = np.zeros((4,3))
        surrounding_locations[0] = self.get_location_base_from_pixel(pixel_u - pixel_bounding, pixel_v, altitude, pitch, yaw)
        surrounding_locations[1] = self.get_location_base_from_pixel(pixel_u + pixel_bounding, pixel_v, altitude, pitch, yaw)
        surrounding_locations[2] = self.get_location_base_from_pixel(pixel_u, pixel_v - pixel_bounding, altitude, pitch, yaw)
        surrounding_locations[3] = self.get_location_base_from_pixel(pixel_u, pixel_v + pixel_bounding, altitude, pitch, yaw)

        diff_locations = surrounding_locations - location
        diff_min_x = np.min(diff_locations[:,0])
        diff_max_x = np.max(diff_locations[:,0])
        diff_min_z = np.min(diff_locations[:,2])
        diff_max_z = np.max(diff_locations[:,2])

        return {"min_diff_z": diff_min_z, "max_diff_z": diff_max_z, "min_diff_x": diff_min_x, "max_diff_x": diff_max_x}




        









