# 🦾 3R Planar Manipulator: Interactive Kinematics Engine


An interactive Forward Kinematics (FK) simulator for a 3-link planar robotic arm. This project demonstrates high-precision coordinate transformation, event-driven programming in Matplotlib, and rigorous software verification through unit testing.

---
# 📖 Technical Report 

For a detailed technical report covering coordinate frame assignment, DH parameter table, and Kinematic modeling via DH convention, please see [Technical Report]()

### 🚀 Key Technical Specifications

*   **DH Parameter Modeling:** Leveraged the Denavit-Hartenberg (DH) convention to define the robot's kinematic chain, using a systematic approach to link frames and joint offsets.
*   **Homogeneous Transformations:** Implemented 4×4 transformation matrices (𝑖_𝑖−1𝑇) using NumPy, allowing for scalable calculation of the end-effector pose by multiplying individual link transforms.
*   **Interactive Simulation:** Users input three target joint angles ($\theta_1, \theta_2, \theta_3$). A **right-click event** triggers a smooth, real-time animation of the arm transitioning from its home position $(0,0,0)$ to the target configuration.
*   **Forward Kinematics Engine:** Implemented trigonometric modeling to map joint space to Cartesian space via DH parameters:
    *   $x = l_1 \cos(\theta_1) + l_2 \cos(\theta_1 + \theta_2) + l_3 \cos(\theta_1 + \theta_2 + \theta_3)$
    *   $y = l_1 \sin(\theta_1) + l_2 \sin(\theta_1 + \theta_2) + l_3 \sin(\theta_1 + \theta_2 + \theta_3)$

*   **Rigorous Verification:** A dedicated **Unit Testing** suite validates the DH-matrix outputs against geometric benchmarks to ensure mathematical integrity.
*   **Dynamic Visualization:** Real-time plotting using `Matplotlib` with an interactive event-loop for animation.

---
### 📐 The Mathematics (DH Convention)

The final pose of the 3rd link (end-effector) is calculated by the product of three individual homogeneous transformation matrices:

$$^0_3T = ^0_1T(\theta_1) \cdot ^1_2T(\theta_2) \cdot ^2_3T(\theta_3)$$

This matrix-based approach ensures the codebase is easily extensible to higher Degree of Freedom (DOF) robotic systems and is compatible with industry-standard simulation configurations.

---


### 🧪 Verification & Testing

This project follows **Test-Driven Development (TDD)** principles to ensure mathematical reliability - a critical requirement for physical robotic systems.

### 📂 Project Structure

*   **`src/main.py`**: Core kinematics engine (DH Parameters) and interactive Matplotlib GUI.
*   **`tests/unit_tests.py`**: Mathematical validation of $4 \times 4$ transformation matrices.
*   **`tests/test_plan.py`**: Functional testing of the event-driven animation and user input.

