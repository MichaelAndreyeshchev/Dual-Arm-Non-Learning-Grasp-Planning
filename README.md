# Input and Output

**Inputs:** The system will use RGB-D image data. Additional sensory data may include tactile information from advanced sensors like GelSight.

**Outputs:** Outputs will include grasp poses for each arm (notated as \(g_l\) for the left arm and \(g_r\) for the right arm), boolean indicators of task completion, and diagnostic information for performance evaluation.

# Method

## Notations

- **Grasp Pose for Left Arm (\(g_l\))**: Represents the specific pose or configuration of the left arm of the robot necessary to grasp an object, including the arm's position and orientation in space.
- **Grasp Pose for Right Arm (\(g_r\))**: Represents the specific pose or configuration of the right arm of the robot necessary to grasp an object, including the arm's position and orientation in space.
- **Available List of Poses (\(G\))**: A set of all feasible grasp poses for the robotic arms, where each pose \(g\) in \(G\) is a tuple \((g_l, g_r)\), with \(g_l\) for the left arm and \(g_r\) for the right arm.
- **Policy (\(\pi\))**: A mapping from the state space \(S\) to the probability of taking each action (grasp pose) in each state. In a discrete policy, \(\pi(s, g)\) denotes the probability of selecting pose \(g\) in state \(s\).
- **Reward Function (\(R\))**: A function \(R(s, g)\) returning the reward received after action \(g\) in state \(s\), reflecting the success of the grasping action.
  - **left_gripper_pos**: Position of the left gripper.
  - **right_gripper_pos**: Position of the right gripper.
  - **object_pos**: Position of the object.
  - **left_gripper_object_dist**: Euclidean distance between the left gripper and the object.
  - **right_gripper_object_dist**: Euclidean distance between the right gripper and the object.

    Then, the reward function \(R\) can be mathematically defined as follows:

    ```
    R = 
    \begin{cases}
        -(\text{left_gripper_object_dist} + \text{right_gripper_object_dist}) + 1.0, & \text{if both distances are } < 0.05, \\
        -(\text{left_gripper_object_dist} + \text{right_gripper_object_dist}), & \text{otherwise}.
    \end{cases}
    ```
    
    where
    
    ```
    \text{left_gripper_object_dist} = \left\lVert \text{left_gripper_pos} - \text{object_pos} \right\rVert_2, \\
    \text{right_gripper_object_dist} = \left\lVert \text{right_gripper_pos} - \text{object_pos} \right\rVert_2,
    ```
    
    and \(\left\lVert \cdot \right\rVert_2\) denotes the Euclidean norm (L2 norm).

- **State (\(S\))**: The state \(s\) in state space \(S\) representing the current environmental observation, including object features from RGB images.

## Non-Learning Process Flow

The process flow for the non-learning approach can be summarized in the following steps:

1. **RGB-D Sensing:** Utilize RGB-D (color and depth) sensors to capture the environmental data, including the position and orientation of the blocks.
2. **Grasp Pose Determination:** Analyze the sensory data to calculate the optimal grasp poses (\(g_l\) and \(g_r\)) for both arms to securely grasp each block.
3. **Robotic Control Framework:** Employ a robotic control framework, such as MoveIt or PyBullet, to plan and execute the arm movements based on the determined grasp poses.
4. **Grasping and Lifting:** Execute the planned movements to grasp the blocks with both arms and then lift them simultaneously, ensuring coordination and stability.

## Grasp Pose Determination Steps

1. **Object Recognition and Feature Extraction:**
   - Let \(D_{\text{RGB-D}}\) represent the RGB-D sensor data.
   - Object detection function: \(O_{\text{detect}}(D_{\text{RGB-D}}) \rightarrow \{B_i\}\), where \(\{B_i\}\) are detected blocks.
   - Feature extraction function: \(F_{\text{extract}}(B_i) \rightarrow (\mathbf{d}_i, \theta_i)\), where \(\mathbf{d}_i\) and \(\theta_i\) are the dimensions and orientation of block \(i\).

2. **Grasp Point Calculation:**
   - Define grasp points calculation function: \(G_{\text{calc}}(\mathbf{d}_i, \theta_i) \rightarrow \mathbf{p}_{\text{grasp},i}\), where \(\mathbf{p}_{\text{grasp},i}\) are the potential grasp points on block \(i\).
   - Stability factors: surface area \(A_i\) and center of mass \(\mathbf{com}_i\).

3. **Reachability Analysis:**
   - Define reachability function for each arm: \(R_{\text{arm}}(\mathbf{p}_{\text{grasp},i}, \text{workspace}) \rightarrow \text{feasible or not}\).
   - Select grasp points \(\mathbf{p}_{\text{grasp},i}\) that are feasible for each arm.

4. **Collision Avoidance and Path Planning:**
   - Trajectory planning function: \(T_{\text{plan}}(\text{current position}, \mathbf{p}_{\text{grasp},i}) \rightarrow \text{path}\).
   - Collision avoidance using RRT or PRM algorithms.

5. **Grasp Pose Optimization:**
   - Define optimization function: \(O_{\text{optimize}}(\text{path}, \mathbf{p}_{\text{grasp},i}) \rightarrow \text{optimized path}\).
   - Adjust paths to maximize stability and minimize slippage or tilting risk.
