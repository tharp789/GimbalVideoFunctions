import numpy as np
import cv2
import math
import os
import sys

CAMERA_VERTICAL_OFFSET = 0.055 #meters
CAMERA_HORIZONTAL_OFFSET = 0.010 #meters

DRONE_TO_BASE_FORWARD_OFFSET = 0.00 #meters
DRONE_TO_BASE_HEIGHT_OFFSET = 0.032 #meters

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)
sys.path.append(current)
sys.path.append(os.path.join(parent_directory, 'vision'))

# Frame convetion is z forward, x right, y down

class ProjectionKinematics:
    def __init__(self, camera_calibration_file_path, drone_to_base_forward_offset_m=DRONE_TO_BASE_FORWARD_OFFSET, drone_to_base_height_offset_m=DRONE_TO_BASE_HEIGHT_OFFSET):
        camera_calibration = np.load(camera_calibration_file_path)
        self.calibration_matrix = camera_calibration['camera_matrix']
        self.distortion = camera_calibration['distortion']
        self.drone_to_base_forward_offset_m = drone_to_base_forward_offset_m
        self.drone_to_base_height_offset_m = drone_to_base_height_offset_m

    def get_base_to_camera_transform(self, yaw, pitch):
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
        x_offset = CAMERA_HORIZONTAL_OFFSET * np.sin(yaw)
        z_offset = CAMERA_HORIZONTAL_OFFSET * np.cos(yaw)
        y_offset = CAMERA_VERTICAL_OFFSET

        cam_transformation[:3,:3] = cam_rot
        cam_transformation[:3,3] = [x_offset, y_offset, z_offset]

        return cam_transformation
    
    def location_drone_to_location_world(self, location_drone, heading, lat, long):
        '''
        Transform the location from the drone frame to the world frame in latitude and longitude
        Drone frame is z forward, x right, y down
        World frame is x east, y north, z up
        '''
        # compass is clockwise angle, converting to counter clockwise angle
        heading_rad = np.radians(360 - heading)
        heading_rotation = np.array([[np.cos(heading_rad), -np.sin(heading_rad), 0], [np.sin(heading_rad), np.cos(heading_rad), 0], [0, 0, 1]])
        # world to drone is a -90 degree rotation around the x axis
        world_to_drone = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])

        rotation_world_to_drone = np.dot(heading_rotation, world_to_drone)
        location_world_xyz = np.dot(rotation_world_to_drone, location_drone)
        # get the horizontal distance, from x and y components
        distance_m = np.linalg.norm(location_world_xyz[:2])

        new_lat, new_long = self.get_destination(lat, long, location_world_xyz[1], location_world_xyz[0])
        return np.array([new_lat, new_long]), distance_m
    
    def location_base_to_location_drone(self, location_base):
        '''
        Transform the location from the base frame to the drone frame
        '''
        location_drone_to_location_base_translation = np.array([0, self.drone_to_base_height_offset_m, self.drone_to_base_forward_offset_m])
        location_drone = location_base + location_drone_to_location_base_translation
        return location_drone
    
    def get_lat_long_from_pixel(self, pixel_u, pixel_v, altitude, pitch, yaw, heading, lat, long):
        '''
        Get the latitude and longitude of a pixel in the world frame

        Parameters:
        pixel_u (int): The x pixel location of the object
        pixel_v (int): The y pixel location of the object
        altitude (float): The altitude of the camera
        pitch (float): The pitch angle of the gimbal
        yaw (float): The yaw angle of the gimbal in degrees
        heading (float): The heading of the drone in absolute coordinates in degrees
        lat (float): The latitude of the drone
        long (float): The longitude of the drone

        Returns:
        numpy.ndarray: The latitude and longitude of the object in the world frame (2,)
        '''

        altitude_base = altitude - self.drone_to_base_height_offset_m
        location_base = self.get_location_base_from_pixel(pixel_u, pixel_v, altitude_base, pitch, yaw)
        location_drone = self.location_base_to_location_drone(location_base)
        location_world, distance_m = self.location_drone_to_location_world(location_drone, heading, lat, long)
        return location_world, distance_m
        
    
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
        base_to_cam = self.get_base_to_camera_transform(yaw_rad, pitch_rad)

        # get the 3D location of the object in the camera frame
        cam_ray = self.get_3d_ray_from_pixel(pixel_u, pixel_v, self.calibration_matrix, self.distortion)
        # camera pitch is global (with gravity), yaw is local to camera initialization which is aligned with drone heading

        # frame transform from base to cam, is the same as transform from points in cam to points in base (or vectors too)
        # reference: http://www.cs.cmu.edu/afs/cs/academic/class/15494-s24/lectures/kinematics/jennifer-kay-kinematics-2020.pdf
        # reference: https://www.dgp.toronto.edu/~neff/teaching/418/transformations/transformation.html
        ray_in_base_frame_shot_from_cam = np.dot(base_to_cam[:3,:3], cam_ray)
        camera_altitude = altitude - CAMERA_VERTICAL_OFFSET
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
    
    def get_destination(self, lat1, lon1, x, y):
        """
        Gets the the destination lat and lon given a starting lat, lon, and x, y displacement in meters
        """
        # Destination point - http://www.movable-type.co.uk/scripts/latlong.html
        # φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
        # λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
        # φ is latitude, λ is longitude, θ is the bearing (clockwise from north),
        # δ is the angular distance d/R; d being the distance travelled, R the earth’s radius

        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)        
        R = 6371000
        d = math.hypot(x, y) / R
        theta = math.atan2(y, x)
        lat2 = math.asin(
            math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(theta)
        )
        lon2 = lon1 + math.atan2(
            math.sin(theta) * math.sin(d) * math.cos(lat1),
            math.cos(d) - math.sin(lat1) * math.sin(lat2),
        )
        lon2 = (lon2 + 3 * math.pi) % (2 * math.pi) - math.pi

        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        return lat2, lon2




        









