import numpy as np

def generate_point_cloud(rgb_image, depth_image, camera_intrinsics):
    """
    Generate a point cloud from an RGB-D image.

    :param rgb_image: The RGB image as a numpy array.
    :param depth_image: The depth image as a numpy array.
    :param camera_intrinsics: The camera intrinsic parameters as a dictionary with fx, fy, cx, cy.
    :return: Point cloud as an Nx6 numpy array (x, y, z, r, g, b).
    """
    # Unpack camera intrinsics
    fx, fy, cx, cy = camera_intrinsics['fx'], camera_intrinsics['fy'], camera_intrinsics['cx'], camera_intrinsics['cy']
    
    # Create a mesh grid of pixel coordinates
    height, width = depth_image.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Back-project the pixel coordinates to 3D space
    z = depth_image.flatten()
    x = (u.flatten() - cx) * z / fx
    y = (v.flatten() - cy) * z / fy
    
    # Stack the coordinates and color
    points = np.vstack((x, y, z)).transpose()
    colors = rgb_image.reshape(-1, rgb_image.shape[2])
    point_cloud = np.hstack((points, colors))
    
    return point_cloud

def main():
    # Example camera intrinsics (these should be adjusted to your camera setup)
    camera_intrinsics = {
        'fx': 525.0,  # Focal length in x axis
        'fy': 525.0,  # Focal length in y axis
        'cx': 319.5,  # Optical center in x axis
        'cy': 239.5   # Optical center in y axis
    }
    
    # Assuming rgb_image and depth_image are available from the previous step
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)  # Placeholder
    depth_image = np.ones((480, 640)) * 1.0  # Placeholder, in meters
    
    point_cloud = generate_point_cloud(rgb_image, depth_image, camera_intrinsics)
    print("Point cloud generated with shape:", point_cloud.shape)

if __name__ == "__main__":
    main()
