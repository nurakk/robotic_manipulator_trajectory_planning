# Robotic Manipulator Trajectory Planning Using Python

## 1. Project Overview

This project demonstrates joint-space and Cartesian-space trajectory planning of a two-degree-of-freedom (2-DOF) planar robotic manipulator using Python.

The simulation generates smooth robotic arm movements using cubic polynomial interpolation. Forward kinematics and inverse kinematics are used to calculate the robot's joint positions and end-effector coordinates.

The project also includes an animated visualization of the robotic manipulator, allowing users to observe and compare the two trajectory-planning approaches.

This experiment is suitable for engineering students studying Robotics, Automation, Mechatronics and Robot Kinematics.

## 2. Objectives

- Understand trajectory planning in robotic manipulators.
- Generate joint-space trajectories.
- Generate Cartesian-space straight-line trajectories.
- Implement cubic polynomial interpolation.
- Apply forward kinematics and inverse kinematics.
- Calculate joint position, velocity and acceleration.
- Visualize robotic arm movement through animation.
- Compare joint-space and Cartesian-space trajectories.

## 3. Algorithms and Methods Used

### 3.1 Cubic Polynomial Interpolation

Cubic polynomial interpolation generates a smooth trajectory with zero initial and final velocity.

\[
s(t)=3(t/T)^2-2(t/T)^3
\]

### 3.2 Joint-Space Trajectory Planning

Joint-space planning calculates the motion of individual robot joints between initial and final joint angles.

\[
q(t)=q_0+(q_f-q_0)s(t)
\]

### 3.3 Cartesian-Space Trajectory Planning

Cartesian-space planning generates a straight-line trajectory for the robot end effector between two Cartesian positions.

### 3.4 Forward Kinematics

Forward kinematics calculates the end-effector position from the robot joint angles.

\[
x=L_1\cos q_1+L_2\cos(q_1+q_2)
\]

\[
y=L_1\sin q_1+L_2\sin(q_1+q_2)
\]

### 3.5 Inverse Kinematics

Inverse kinematics calculates the required joint angles for a specified end-effector position.

## 4. Robot Configuration

The simulation uses a 2-DOF planar robotic manipulator.

- Number of joints: 2
- Number of links: 2
- Link 1 length: 1 metre
- Link 2 length: 1 metre
- Initial joint angles: 30° and 60°
- Final joint angles: 60° and 30°
- Trajectory duration: 5 seconds

These parameters can be modified in the Python program.

## 5. Technologies Used

- Python 3
- Jupyter Notebook
- NumPy
- Matplotlib
- Matplotlib Animation
- Pillow

## 6. Software Requirements

Install the required Python libraries:

```python
%pip install numpy matplotlib pillow
```

## 7. How to Run the Project

### Step 1: Clone the repository

```bash
git clone https://github.com/nurakk/robotic_manipulator_trajectory_planning.git
```

### Step 2: Navigate to the repository

```bash
cd robotic_manipulator_trajectory_planning
```

### Step 3: Install dependencies

```bash
pip install -r requirements.txt
```

### Step 4: Run the Python program

```bash
python robotic_manipulator_trajectory_planning.py
```

Alternatively, open the Jupyter Notebook:

```bash
jupyter notebook
```

Open `robotic_manipulator_trajectory_planning.ipynb` and execute the cells sequentially.

## 8. Expected Outputs

The program generates:

1. Joint position versus time graph.
2. Joint velocity versus time graph.
3. Joint acceleration versus time graph.
4. Joint-space end-effector trajectory.
5. Cartesian-space straight-line trajectory.
6. Comparison of both trajectory-planning methods.
7. Animated 2-DOF robotic manipulator.
8. End-effector trajectory trails.
9. Optional GIF animation export.

## 9. Animation

The animation displays two robotic manipulators side by side:

- Left: Joint-space trajectory.
- Right: Cartesian-space trajectory.

The end-effector trail illustrates the path followed by the robot during movement.

The animation can also be exported as a GIF using Matplotlib's Pillow writer.

## 10. Repository Structure

```text
robotic_manipulator_trajectory_planning/
│
├── robotic_manipulator_trajectory_planning.py
├── robotic_manipulator_trajectory_planning.ipynb
├── robot_trajectory.gif
├── requirements.txt
├── README.md
└── .gitignore
```

## 11. Applications

- Industrial robotic manipulators
- Pick-and-place operations
- Robotic welding
- Material handling
- Robot motion control
- Automated manufacturing
- Educational robotics simulation

## 12. Practical Exercises

1. Change the initial and final joint angles.
2. Change the length of both robot links.
3. Modify the total trajectory execution time.
4. Generate different joint-space trajectories.
5. Compare joint-space and Cartesian-space paths.
6. Save the robot animation as a GIF.
7. Observe how joint velocity and acceleration change with trajectory duration.

## 13. Limitations

- The simulation considers a planar 2-DOF robotic manipulator.
- Joint limits and collision avoidance are not implemented.
- Robot dynamics and torque calculations are not included.
- This is a software simulation and does not directly control physical hardware.

## 14. Author

**Arun Kumar K**  
Department of Robotics and Automation

GitHub: [@nurakk](https://github.com/nurakk)

## 15. References

- [NumPy Documentation](https://numpy.org/doc/)
- [Matplotlib Documentation](https://matplotlib.org/stable/)
- [Matplotlib Animation Documentation](https://matplotlib.org/stable/api/animation_api.html)
- [Robotics: Modelling, Planning and Control](https://link.springer.com/book/10.1007/978-1-84628-642-1)

---

**Educational Project | Robotics and Automation Laboratory**
