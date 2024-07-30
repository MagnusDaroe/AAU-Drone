from mpc_module import NMPC
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D












if False:
    mpc = NMPC("drone")

    traj = np.array([10, 10, 10])
    x = np.array([0, 0, 0])

    for i in range(10):
        u_list, x_list = mpc.controller(x, traj)

    # Plotting the trajectory
    x_list = np.array(x_list)
    print(np.shape(x_list[0]))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x_list[:, 0], x_list[:, 1], x_list[:, 2], 'r')
    ax.plot([traj[0]], [traj[1]], [traj[2]], 'bo')

    # Add labels for better understanding
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
