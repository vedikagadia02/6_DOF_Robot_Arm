import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from roboticstoolbox import DHRobot, RevoluteDH
from itertools import product

# Define the robot
robot = DHRobot([
    RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[0, 2*np.pi], offset=2.9),
    RevoluteDH(d=0.5, a=2, alpha=0, qlim=[-np.pi/2, np.pi]),
    RevoluteDH(d=0, a=-0.6, alpha=-np.pi/2, qlim=[0, 2*np.pi]),
    RevoluteDH(d=0, a=0.875, alpha=-np.pi/2, qlim=[0, 2*np.pi]),
    RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-np.pi/2, np.pi/2]),
    RevoluteDH(d=0, a=1.3, alpha=0, qlim=[0, 2*np.pi]),
], name='MyCobotRobot')

# Define joint values
joint_values = [
    np.arange(0, 2*np.pi + np.pi/4, np.pi/4),               # Joint 1
    np.arange(-np.pi/2, np.pi + np.pi/4, np.pi/4),           # Joint 2
    np.arange(0, 2*np.pi + np.pi/4, np.pi/4),               # Joint 3
    np.arange(0, 2*np.pi + np.pi/4, np.pi/4),               # Joint 4
    np.arange(-np.pi/2, np.pi/2 + np.pi/4, np.pi/4),         # Joint 5
    np.arange(0, 2*np.pi + np.pi/4, np.pi/4)                # Joint 6
]

# Generate all possible combinations of joint values
joint_combinations = list(product(*joint_values))

# Initialize arrays to store end effector positions
end_effector_positions = np.zeros((len(joint_combinations), 3))

# Compute end effector positions for each joint configuration
for i, joint_config in enumerate(joint_combinations):
    end_effector_pose = robot.fkine(joint_config)
    end_effector_positions[i, :] = end_effector_pose.t

# Randomly select 10,000 points and plot them with varying colors and transparency
num_points_to_plot = 5000
selected_indices = np.random.choice(end_effector_positions.shape[0], num_points_to_plot, replace=False)
selected_points = end_effector_positions[selected_indices, :]

# Plot the selected points with varying colors and transparency
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(selected_points[:, 0], selected_points[:, 1], selected_points[:, 2], s=10, c='r', marker='o', alpha=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Randomly Selected Points in Robot Workspace')

# Allow interactive 3D rotation
ax.view_init(elev=20, azim=30)  # Set initial view angle
plt.show()
