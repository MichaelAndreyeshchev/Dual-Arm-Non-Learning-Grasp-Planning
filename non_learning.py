import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler

#from tensorflow import data as tf_data
#from keras import ops
#import keras
#from keras import layers
import matplotlib.pyplot as plt


# Initialize PyBullet Simulation
def initialize_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")

    # Load Robot Arms and Objects
    panda_id1 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    panda_id2 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, basePosition=[0, 0.5, 0])
    block_id1 = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0.2, 0.5])
    block_id2 = p.loadURDF("cube_small.urdf", basePosition=[0.5, -0.2, 0.5])
    return plane_id, panda_id1, panda_id2, block_id1, block_id2

# Capture RGB-D Images

def capture_rgbd_images(camera_position, camera_orientation, width=640, height=480, fov=60, aspect=1, nearVal=0.1, farVal=100):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camera_position, distance=1.0,
                                                      yaw=camera_orientation[0], pitch=camera_orientation[1],
                                                      roll=camera_orientation[2], upAxisIndex=2)
    # Compute projection matrix
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearVal, farVal)
    
    # Capture image
    _, _, rgb_img, depth_img, _ = p.getCameraImage(width=width, height=height,
                                                   viewMatrix=view_matrix,
                                                   projectionMatrix=projection_matrix)
    
    # Reshape RGB image
    rgb_array = np.reshape(np.array(rgb_img, dtype=np.uint8), (height, width, 4))
    rgb_array = rgb_array[:, :, :3]  # Drop the alpha channel
    
    # Convert depth image
    depth_buffer = np.reshape(np.array(depth_img), (height, width))
    depth_array = farVal * nearVal / (farVal - (farVal - nearVal) * depth_buffer)
    
    return rgb_array, depth_array

# Generate Point Cloud from Depth Image
def depth_image_to_point_cloud(depth_img, camera_intrinsics):
    h, w = depth_img.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')
    x = (i - camera_intrinsics['cx']) * depth_img / camera_intrinsics['fx']
    y = (j - camera_intrinsics['cy']) * depth_img / camera_intrinsics['fy']
    z = depth_img
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    return points


def depth_image_to_point_cloud(depth_img, camera_intrinsics):
    """
    Convert a depth image into a point cloud.
    
    Parameters:
    - depth_img: numpy array of the depth image.
    - camera_intrinsics: Dictionary with keys 'fx', 'fy', 'cx', 'cy'.
    
    Returns:
    - points: (N, 3) numpy array of 3D point coordinates.
    """
    height, width = depth_img.shape
    fx, fy = camera_intrinsics['fx'], camera_intrinsics['fy']
    cx, cy = camera_intrinsics['cx'], camera_intrinsics['cy']
    
    # Create a grid of coordinates corresponding to the indices of the depth image
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Backproject depth to 3D points in camera coordinates
    x = (u - cx) * depth_img / fx
    y = (v - cy) * depth_img / fy
    z = depth_img
    
    # Stack to get (N, 3) array
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    
    return points


def depth_to_point_cloud(depth_img, camera_intrinsics):
    """
    Convert a depth image into a point cloud.
    
    Parameters:
    - depth_img: numpy array of the depth image.
    - camera_intrinsics: Dictionary with keys 'fx', 'fy', 'cx', 'cy'.
    
    Returns:
    - points: (N, 3) numpy array of 3D point coordinates.
    """
    height, width = depth_img.shape
    fx, fy = camera_intrinsics['fx'], camera_intrinsics['fy']
    cx, cy = camera_intrinsics['cx'], camera_intrinsics['cy']
    
    # Create a grid of coordinates corresponding to the indices of the depth image
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Backproject depth to 3D points in camera coordinates
    x = (u - cx) * depth_img / fx
    y = (v - cy) * depth_img / fy
    z = depth_img
    
    # Stack to get (N, 3) array
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    
    return points

def append_rgb_to_point_cloud(point_cloud, rgb_img):
    """
    Append RGB values to each point in the point cloud.
    
    Parameters:
    - point_cloud: (N, 3) numpy array of 3D point coordinates.
    - rgb_img: (H, W, 3) numpy array of the RGB image.
    
    Returns:
    - points_with_rgb: (N, 6) numpy array of points with RGB values appended.
    """
    height, width, _ = rgb_img.shape
    # Flatten the RGB image to match the point cloud's shape
    rgb_flat = rgb_img.reshape(-1, 3)
    
    # Concatenate point coordinates with their corresponding RGB values
    points_with_rgb = np.concatenate((point_cloud, rgb_flat), axis=1)
    
    return points_with_rgb

def segment_cubes(point_cloud, z_threshold=0.5):
    """
    Segment the point cloud to isolate points that are part of the cubes based on a Z-coordinate threshold.
    
    Parameters:
    - point_cloud: (N, 3) numpy array of 3D point coordinates.
    - z_threshold: Z-coordinate threshold to isolate the cubes.
    
    Returns:
    - cube_points: (M, 3) numpy array of 3D points that are part of the cubes.
   
    """
    # Assume that points with a Z-coordinate above a certain threshold belong to the cubes
    cube_points = point_cloud[point_cloud[:, 2] >= z_threshold]

    return cube_points


def preprocess_data_for_pointnet(points, sample_size=1024, normalize=True, center=True):
    """
    Preprocess point cloud data for PointNet++.

    Parameters:
    - points: numpy.ndarray, the original point cloud data of shape (N, 3) where N is the number of points.
    - sample_size: int, the number of points to sample from the original point cloud for downsampling.
    - normalize: bool, whether to normalize the point cloud data.
    - center: bool, whether to center the point cloud data around the origin.

    Returns:
    - processed_points: numpy.ndarray, the preprocessed point cloud data of shape (sample_size, 3).
    """
    # Downsampling or random sampling
    if points.shape[0] > sample_size:
        indices = np.random.choice(points.shape[0], sample_size, replace=False)
        sampled_points = points[indices, :]
    else:
        sampled_points = points

    # Centering the data
    if center:
        centroid = np.mean(sampled_points, axis=0)
        sampled_points -= centroid

    # Normalization
    if normalize:
        scaler = StandardScaler()
        sampled_points = scaler.fit_transform(sampled_points)

    processed_points = sampled_points
    return processed_points

# Predict Grasp Poses with PointNet++
def predict_grasp_poses(points):
    # Placeholder function for grasp pose prediction
    # Assume this function interfaces with a trained PointNet++ model
    grasp_poses = np.array([[0.5, 0.0, 0.5, 0, 0, 0], [0.5, 0.5, 0.5, 0, 0, 0]])  # Dummy grasp poses
    return grasp_poses

# Execute Grasps in PyBullet
def execute_grasps(grasp_poses):
    for pose in grasp_poses:
        position, orientation = pose[:3], pose[3:]
        # Convert Euler angles to quaternion for PyBullet
        quat = R.from_euler('xyz', orientation).as_quat()
        # Placeholder for moving the robot arm to the grasp position and executing the grasp
        # This would involve inverse kinematics and control commands


def visualize_point_cloud(points):
    """
    Visualize a 3D point cloud using Matplotlib.

    Parameters:
    - points: (N, 3) numpy array of 3D point coordinates.
    """
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

# Example usage:
# Initialize simulation and capture images
initialize_simulation()

position1 = np.array([0.5, 0.2, 0.5])  # Position of the first cuboid
position2 = np.array([0.5, -0.2, 0.5]) # Position of the second cuboid
midpoint = (position1 + position2) / 2

camera_distance = 1.5  # Adjust as needed for a good view
camera_height = 1.0    # Adjust as needed for elevation
camera_position = np.array([midpoint[0], midpoint[1], camera_height])

# Calculate the yaw angle to face the midpoint
# Assuming the camera is placed along the x-axis, looking towards the origin
yaw = np.arctan2(midpoint[1] - camera_position[1], midpoint[0] - camera_position[0])
yaw = np.degrees(yaw)  # Convert to degrees if necessary

# Set pitch to point downwards
pitch = -10  # This is a small negative angle; adjust as needed

# Assuming no roll is needed
roll = 0

camera_orientation = [yaw, pitch, roll]
rgb_array, depth_array = capture_rgbd_images(camera_position=camera_position, camera_orientation=camera_orientation)

# Generate point cloud and append RGB values
point_cloud = depth_image_to_point_cloud(depth_array, camera_intrinsics={'fx': 525, 'fy': 525, 'cx': 320, 'cy': 240})
point_cloud_with_rgb = append_rgb_to_point_cloud(point_cloud, rgb_array)

# Segment cubes and preprocess for PointNet++
cube_points = segment_cubes(point_cloud_with_rgb, z_threshold=0.5)
preprocessed_cube_points = preprocess_data_for_pointnet(cube_points)
np.save('contact_graspnet_pytorch/point_cloud.npy', preprocessed_cube_points)

# Visualize the segmented cubes point cloud
visualize_point_cloud(preprocessed_cube_points)

"""
initialize_simulation()

position1 = np.array([0.5, 0.2, 0.5])  # Position of the first cuboid
position2 = np.array([0.5, -0.2, 0.5]) # Position of the second cuboid
midpoint = (position1 + position2) / 2

camera_distance = 1.5  # Adjust as needed for a good view
camera_height = 1.0    # Adjust as needed for elevation
camera_position = np.array([midpoint[0], midpoint[1], camera_height])

# Calculate the yaw angle to face the midpoint
# Assuming the camera is placed along the x-axis, looking towards the origin
yaw = np.arctan2(midpoint[1] - camera_position[1], midpoint[0] - camera_position[0])
yaw = np.degrees(yaw)  # Convert to degrees if necessary

# Set pitch to point downwards
pitch = -10  # This is a small negative angle; adjust as needed

# Assuming no roll is needed
roll = 0

camera_orientation = [yaw, pitch, roll]


view_matrix = p.computeViewMatrixFromYawPitchRoll(camera_position, distance=camera_distance,
                                                  yaw=camera_orientation[0], pitch=camera_orientation[1],
                                                  roll=camera_orientation[2], upAxisIndex=2)

projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1, nearVal=0.1, farVal=100)
width, height, rgb_img, depth_img, seg_img = p.getCameraImage(width=640, height=480,
                                                              viewMatrix=view_matrix,
                                                              projectionMatrix=projection_matrix)

width, height = 640, 480  # The width and height used in the p.getCameraImage function
rgba_array = np.reshape(rgb_img, (height, width, 4))

# Convert the RGBA image to RGB by discarding the alpha channel
rgb_array = rgba_array[:, :, :3]

# Display the image using matplotlib
plt.imshow(rgb_array)
plt.axis('off')  # Hide the axes
plt.show()

position1 = np.array([0.5, 0.2, 0.5])  # Position of the first cuboid
position2 = np.array([0.5, -0.2, 0.5]) # Position of the second cuboid
midpoint = (position1 + position2) / 2

camera_distance = 1.5  # Adjust as needed for a good view
camera_height = 1.0    # Adjust as needed for elevation
camera_position = np.array([midpoint[0], midpoint[1], camera_height])

# Calculate the yaw angle to face the midpoint
# Assuming the camera is placed along the x-axis, looking towards the origin
yaw = np.arctan2(midpoint[1] - camera_position[1], midpoint[0] - camera_position[0])
yaw = np.degrees(yaw)  # Convert to degrees if necessary

# Set pitch to point downwards
pitch = -10  # This is a small negative angle; adjust as needed

# Assuming no roll is needed
roll = 0

camera_orientation = [yaw, pitch, roll]

rgb_img, depth_img = capture_rgbd_images(camera_position=camera_position, camera_orientation=camera_orientation)

#rgb_img, depth_img = capture_rgbd_images(camera_position=[1, 1, 1], camera_orientation=[0, -45, 0])

width, height = 640, 480  # These should match the width and height used in capture_rgbd_images
print(len(rgb_img))

camera_intrinsics = {'fx': 525, 'fy': 525, 'cx': 319.5, 'cy': 239.5}  # Example values

# Convert depth image to point cloud
point_cloud = depth_to_point_cloud(depth_img, camera_intrinsics)
"""

"""

# Visualization of the RGB image
#plt.imshow(rgb_img, interpolation='nearest')
#plt.title('RGB Image')
#plt.show()

# Visualization of the Depth image (assuming depth_img needs normalization for visualization)
#normalized_depth_img = (depth_img - np.min(depth_img)) / (np.max(depth_img) - np.min(depth_img))
#plt.imshow(depth_img, cmap='gray')
#plt.title('Depth Image')
#plt.show()

#camera_intrinsics = {
#    'fx': 525.0,  # Focal length in pixels
#    'fy': 525.0,
#    'cx': 640.0 / 2,  # Assuming width = 640
#    'cy': 480.0 / 2   # Assuming height = 480
#}
#camera_position = [1, 1, 1]
#camera_orientation = [0, -45, 0]  # Example orientation

#points_world = depth_image_to_point_cloud(depth_img, camera_intrinsics, camera_position, camera_orientation)
#from mpl_toolkits.mplot3d import Axes3D

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter(points_world[:, 0], points_world[:, 1], points_world[:, 2], s=1)
#plt.show()

# Commented out the execution of functions to adhere to instructions
#initialize_simulation()
#rgb_img, depth_img = capture_rgbd_images([1, 0, 1], [180, 0, 0])
#points = depth_image_to_point_cloud(depth_img, {'fx': 525, 'fy': 525, 'cx': 319.5, 'cy': 239.5})
#processed_points = preprocess_data_for_pointnet(points)
#grasp_poses = predict_grasp_poses(processed_points)
#execute_grasps(grasp_poses)
"""