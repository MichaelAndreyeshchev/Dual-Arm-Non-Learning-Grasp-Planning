import pybullet as p
import pybullet_data

def setup_environment():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load the plane
    plane = p.loadURDF("plane.urdf")
    
    # Load two Panda arms
    pandaId1 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    pandaId2 = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True, basePosition=[0, 0.5, 0])
    
    # Load two small cubes
    blockId1 = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0.2, 0.5])
    blockId2 = p.loadURDF("cube_small.urdf", basePosition=[0.5, -0.2, 0.5])
    
    print("Environment setup complete. Objects loaded.")

if __name__ == "__main__":
    setup_environment()
