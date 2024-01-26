### NOTES: ###
### * DO NOT USE THREE CAMERAS AT FIRST --> USE A TOP DOWN CAMERA AT FIRST.  SCALE IN THE FUTURE MAYBE. 

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time  # Import time module

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pandaId1 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
pandaId2 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, basePosition=[0,1,0])
plane = p.loadURDF("plane.urdf")
blockId1 = p.loadURDF("cube_small.urdf", basePosition=[0.5,0.5,0])
blockId2 = p.loadURDF("cube_small.urdf", basePosition=[0.5,0.5,0])


def capture_rgbd_image():
    camera_width = 640
    camera_height = 480
    fov = 60
    aspect = camera_width / camera_height
    near_val = 0.1
    far_val = 1.5

    def get_robot_arm_camera_view_matrix(robot_id, end_effector_index, distance=0.3):
        end_effector_state = p.getLinkState(robot_id, end_effector_index)
        end_effector_pos, end_effector_ori = end_effector_state[0], end_effector_state[1]
        return p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=end_effector_pos,
            distance=distance,
            yaw=0,
            pitch=-90,  # Pointing downwards
            roll=0,
            upAxisIndex=2
        )
    
    proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near_val, far_val)

    # Camera 1: Attached to the first Panda robot's arm
    #camera1_view_matrix = get_robot_arm_camera_view_matrix(pandaId1, 11)  # End effector link index for Panda
    #camera1_img = p.getCameraImage(camera_width, camera_height, camera1_view_matrix, proj_matrix)

    # Camera 2: Attached to the second Panda robot's arm
    #camera2_view_matrix = get_robot_arm_camera_view_matrix(pandaId2, 11)  # End effector link index for Panda
    #camera2_img = p.getCameraImage(camera_width, camera_height, camera2_view_matrix, proj_matrix)

    # Camera 3: Static camera looking down on the scene
    camera3_view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0.5, 0.5, 0],  # Center of the scene
        distance=1.0,  # Height from the scene
        yaw=180,
        pitch=-90,  # Pointing down
        roll=0,
        upAxisIndex=2
    )
    camera3_img = p.getCameraImage(camera_width, camera_height, camera3_view_matrix, proj_matrix)

    return camera3_img

def detect_objects(rgbd_image):
    rgb_image = rgbd_image[0]
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

    # Define color range for detection
    lower_color = np.array([30, 150, 50])
    upper_color = np.array([255, 255, 180])

    # Threshold the HSV image to get only the colors in the range
    mask = cv2.inRange(hsv_image, lower_color, upper_color)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detected_objects = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            detected_objects.append((cx, cy))
    
    return detected_objects


def determine_grasp_poses(detected_objects):
    # Analyze objects to determine grasp poses
    pass

def plan_path(grasp_poses):
    # Implement or use existing algorithms like RRT or PRM for path planning
    # Ensure collision avoidance
    pass

def execute_grasp(planned_paths):
    # Use PyBullet commands to control the robot arms
    # Execute the planned paths to grasp and lift the objects
    pass

# def main():
#     while True:
#         rgbd_image = capture_rgbd_image()
#         detected_objects = detect_objects(rgbd_image)
#         grasp_poses = determine_grasp_poses(detected_objects)
#         planned_paths = plan_path(grasp_poses)
#         execute_grasp(planned_paths)

# if __name__ == "__main__":
#     main()



# Keep the simulation running until an interrupt (like pressing CTRL+C)
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)  # Simulation step delay
except KeyboardInterrupt:
    p.disconnect()
    print("Simulation stopped")