#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML, display

# =====================================================
# 1. ROBOT PARAMETERS
# =====================================================

L1 = 1.0                         # Link 1 length (m)
L2 = 1.0                         # Link 2 length (m)

T = 5                            # Total time (seconds)
N = 100                          # Number of trajectory points

time = np.linspace(0, T, N)
tau = time / T

# Initial and final joint angles (degrees)
q_start = np.radians([30, 60])
q_end = np.radians([60, 30])

# =====================================================
# 2. CUBIC POLYNOMIAL INTERPOLATION
# =====================================================

s = 3 * tau**2 - 2 * tau**3

# Joint-space trajectories
q1 = q_start[0] + (q_end[0] - q_start[0]) * s
q2 = q_start[1] + (q_end[1] - q_start[1]) * s

# Joint velocities
dq1 = np.gradient(q1, time)
dq2 = np.gradient(q2, time)

# Joint accelerations
ddq1 = np.gradient(dq1, time)
ddq2 = np.gradient(dq2, time)

# =====================================================
# 3. FORWARD KINEMATICS
# =====================================================

def forward_kinematics(q1, q2):

    x1 = L1 * np.cos(q1)
    y1 = L1 * np.sin(q1)

    x2 = x1 + L2 * np.cos(q1 + q2)
    y2 = y1 + L2 * np.sin(q1 + q2)

    return x1, y1, x2, y2


# Joint-space end-effector positions
x_joint = []
y_joint = []

for i in range(N):

    _, _, x, y = forward_kinematics(q1[i], q2[i])

    x_joint.append(x)
    y_joint.append(y)

x_joint = np.array(x_joint)
y_joint = np.array(y_joint)

# =====================================================
# 4. CARTESIAN-SPACE TRAJECTORY
# =====================================================

# Initial and final end-effector positions
_, _, x_start, y_start = forward_kinematics(
    q_start[0], q_start[1]
)

_, _, x_end, y_end = forward_kinematics(
    q_end[0], q_end[1]
)

# Straight-line Cartesian trajectory
x_cart = x_start + (x_end - x_start) * s
y_cart = y_start + (y_end - y_start) * s

# =====================================================
# 5. INVERSE KINEMATICS
# =====================================================

q1_cart = []
q2_cart = []

for x, y in zip(x_cart, y_cart):

    cos_q2 = (
        x**2 + y**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Check workspace limits
    if cos_q2 < -1 or cos_q2 > 1:
        raise ValueError("Cartesian point is outside robot workspace")

    angle_q2 = np.arccos(cos_q2)

    angle_q1 = (
        np.arctan2(y, x)
        - np.arctan2(
            L2 * np.sin(angle_q2),
            L1 + L2 * np.cos(angle_q2)
        )
    )

    q1_cart.append(angle_q1)
    q2_cart.append(angle_q2)

q1_cart = np.array(q1_cart)
q2_cart = np.array(q2_cart)

# =====================================================
# 6. DISPLAY NUMERICAL RESULTS
# =====================================================

print("ROBOTIC MANIPULATOR TRAJECTORY PLANNING")
print("------------------------------------------")

print("Initial Joint Angles:",
      np.round(np.degrees(q_start), 2), "degrees")

print("Final Joint Angles:",
      np.round(np.degrees(q_end), 2), "degrees")

print("\nInitial End-Effector Position:")
print("X =", round(x_start, 3), "m")
print("Y =", round(y_start, 3), "m")

print("\nFinal End-Effector Position:")
print("X =", round(x_end, 3), "m")
print("Y =", round(y_end, 3), "m")

print("\nTrajectory Generation Completed.")

# =====================================================
# 7. JOINT POSITION, VELOCITY AND ACCELERATION
# =====================================================

fig1, axes = plt.subplots(3, 1, figsize=(10, 9))

axes[0].plot(time, np.degrees(q1), label="Joint 1")
axes[0].plot(time, np.degrees(q2), label="Joint 2")
axes[0].set_title("Joint Position")
axes[0].set_ylabel("Angle (degrees)")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, np.degrees(dq1), label="Joint 1")
axes[1].plot(time, np.degrees(dq2), label="Joint 2")
axes[1].set_title("Joint Velocity")
axes[1].set_ylabel("Velocity (degrees/s)")
axes[1].legend()
axes[1].grid(True)

axes[2].plot(time, np.degrees(ddq1), label="Joint 1")
axes[2].plot(time, np.degrees(ddq2), label="Joint 2")
axes[2].set_title("Joint Acceleration")
axes[2].set_xlabel("Time (seconds)")
axes[2].set_ylabel("Acceleration (degrees/s²)")
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.show()

# =====================================================
# 8. TRAJECTORY COMPARISON
# =====================================================

plt.figure(figsize=(8, 7))

plt.plot(
    x_joint, y_joint,
    "--", linewidth=2,
    label="Joint-Space Path"
)

plt.plot(
    x_cart, y_cart,
    linewidth=2,
    label="Cartesian-Space Path"
)

plt.scatter(x_start, y_start, s=80, label="Start")
plt.scatter(x_end, y_end, s=80, label="End")

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("End-Effector Trajectory Comparison")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

# =====================================================
# 9. ROBOT ANIMATION
# =====================================================

fig, (ax1, ax2) = plt.subplots(
    1, 2, figsize=(12, 6)
)

fig.suptitle(
    "2-DOF Robotic Manipulator Trajectory Planning",
    fontsize=14
)

# Configure both animation panels
for ax in [ax1, ax2]:

    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-2.2, 2.2)

    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")

    ax.set_aspect("equal")
    ax.grid(True)

ax1.set_title("Joint-Space Motion")
ax2.set_title("Cartesian-Space Motion")

# Robot links
robot1, = ax1.plot([], [], "o-", linewidth=5, markersize=8)
robot2, = ax2.plot([], [], "o-", linewidth=5, markersize=8)

# End-effector trails
trail1, = ax1.plot([], [], "--", linewidth=1.5)
trail2, = ax2.plot([], [], "--", linewidth=1.5)

# Target positions
ax1.scatter(x_end, y_end, marker="x", s=100)
ax2.scatter(x_end, y_end, marker="x", s=100)

# Time display
time_text1 = ax1.text(-2.0, 2.0, "")
time_text2 = ax2.text(-2.0, 2.0, "")

# Animation initialization
def init():

    robot1.set_data([], [])
    robot2.set_data([], [])

    trail1.set_data([], [])
    trail2.set_data([], [])

    time_text1.set_text("")
    time_text2.set_text("")

    return (
        robot1, robot2,
        trail1, trail2,
        time_text1, time_text2
    )

# Animation update function
def update(frame):

    # Joint-space robot configuration
    x0 = 0
    y0 = 0

    x1 = L1 * np.cos(q1[frame])
    y1 = L1 * np.sin(q1[frame])

    x2 = x1 + L2 * np.cos(q1[frame] + q2[frame])
    y2 = y1 + L2 * np.sin(q1[frame] + q2[frame])

    robot1.set_data(
        [x0, x1, x2],
        [y0, y1, y2]
    )

    trail1.set_data(
        x_joint[:frame + 1],
        y_joint[:frame + 1]
    )

    # Cartesian-space robot configuration
    xc1 = L1 * np.cos(q1_cart[frame])
    yc1 = L1 * np.sin(q1_cart[frame])

    xc2 = (
        xc1 + L2 * np.cos(
            q1_cart[frame] + q2_cart[frame]
        )
    )

    yc2 = (
        yc1 + L2 * np.sin(
            q1_cart[frame] + q2_cart[frame]
        )
    )

    robot2.set_data(
        [x0, xc1, xc2],
        [y0, yc1, yc2]
    )

    trail2.set_data(
        x_cart[:frame + 1],
        y_cart[:frame + 1]
    )

    # Update time
    time_text1.set_text(
        f"Time: {time[frame]:.2f} s"
    )

    time_text2.set_text(
        f"Time: {time[frame]:.2f} s"
    )

    return (
        robot1, robot2,
        trail1, trail2,
        time_text1, time_text2
    )

# Create animation
ani = FuncAnimation(
    fig,
    update,
    frames=N,
    init_func=init,
    interval=50,
    blit=True,
    repeat=True
)

plt.close(fig)

# Display animation in Jupyter Notebook
display(HTML(ani.to_jshtml()))

# =====================================================
# 10. SAVE ANIMATION AS GIF
# =====================================================

# Uncomment the following line to save the animation

# ani.save(
#     "robot_trajectory.gif",
#     writer="pillow",
#     fps=20
# )

print("\nRobot trajectory animation generated successfully!")
print("Uncomment the GIF save command to export the animation.")


# In[ ]:




