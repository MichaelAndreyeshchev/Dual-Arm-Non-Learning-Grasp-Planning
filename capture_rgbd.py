import pybullet as p
import numpy as np

def capture_rgbd_image(camera_position, target_position, up_vector=[0, 0, 1], image_width=640, image_height=480):
    """
    Capture an RGB-D image from the simulation.
    
    :param camera_position: The position of the camera in the simulation.
    :param target_position: Where the camera is pointed at.
    :param up_vector: The up vector for the camera, for orientation.
    :param image_width: Width of the captured image.
    :param image_height: Height of the captured image.
    :return: RGB image, Depth image
    """
    # Camera settings
    view_matrix = p.computeViewMatrix(camera_position, target_position, up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(image_width) / image_height, nearVal=0.1, farVal=100.0)
    
    # Capture image
    width, height, rgb_img, depth_img, _ = p.getCameraImage(width=image_width, height=image_height, viewMatrix=view_matrix, projectionMatrix=projection_matrix)
    
    # Convert depth into absolute depth
    near = 0.1
    far = 100.0
    depth_img = far * near / (far - (far - near) * depth_img)
    
    return np.array(rgb_img), depth_img

def main():
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    planeId = p.loadURDF("plane.urdf")
    cubeId = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.5])

    camera_position = [1, 1, 1]
    target_position = [0.5, 0, 0.5]
    up_vector = [0, 0, 1]
    view_matrix = p.computeViewMatrix(camera_position, target_position, up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100.0)
    
    width, height, rgb_img, depth_img, _ = p.getCameraImage(width=640, height=480, viewMatrix=view_matrix, projectionMatrix=projection_matrix)

    print("Image captured successfully.")

if __name__ == "__main__":
    main()
