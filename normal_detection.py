import numpy as np
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from sklearn.linear_model import RANSACRegressor
from mpl_toolkits.mplot3d import Axes3D

class Detection:
    def __init__(self,
                 contact_id,
                 drop_in_id,
                 contact_pos,
                 x_num,
                 y_num,
                 pos_cup):
        self.contact_id = contact_id
        self.drop_in_id = drop_in_id
        self.contact_pos = contact_pos
        self.x_num = x_num
        self.y_num = y_num
        self.pos_cup = pos_cup

    def outline(self, position_cup):
        outfit_cup = []
        for num_now in position_cup:
            flag = [1, 1, 1, 1]
            for footprint in position_cup:
                if footprint[0] == num_now[0]:
                    if footprint[1] < num_now[1]:
                        flag[0] = 0
                    elif footprint[1] > num_now[1]:
                        flag[1] = 0
                elif footprint[1] == num_now[1]:
                    if footprint[0] < num_now[0]:
                        flag[2] = 0
                    elif footprint[0] > num_now[0]:
                        flag[3] = 0
            if flag[0] == 1 or flag[1] == 1 or flag[2] == 1 or flag[3] == 1:
                outfit_cup.append(num_now)
        return outfit_cup

    def outline_add(self, position_cup):
        outfit_cup = []
        for num_now in position_cup:
            flag = [1, 1, 1, 1]
            for footprint in position_cup:
                if footprint[0] == num_now[0]:
                    if footprint[1] < num_now[1]:
                        flag[0] = 0
                    elif footprint[1] > num_now[1]:
                        flag[1] = 0
                elif footprint[1] == num_now[1]:
                    if footprint[0] < num_now[0]:
                        flag[2] = 0
                    elif footprint[0] > num_now[0]:
                        flag[3] = 0
            if flag[0] == 1:
                outfit_cup.append([num_now[0], num_now[1] - 1])
                outfit_cup.append(num_now)
            if flag[1] == 1:
                outfit_cup.append([num_now[0], num_now[1] + 1])
                outfit_cup.append(num_now)
            if flag[2] == 1:
                outfit_cup.append([num_now[0] - 1, num_now[1]])
                outfit_cup.append(num_now)
            if flag[3] == 1:
                outfit_cup.append([num_now[0] + 1, num_now[1]])
                outfit_cup.append(num_now)
        return outfit_cup

    def draw_spheres(self, y_num, x_num, outfit, outfit_):
        # Set up the figure
        fig, ax = plt.subplots()

        # Size of the grid
        rows, cols = y_num, x_num

        # Drawing the circles
        for i in range(rows):  # 11
            for j in range(cols):  # 18
                # if [j, i] in outfit:
                #     circle = Circle((j, i), 0.4, color='red')
                #     ax.add_patch(circle)
                # else:
                circle = Circle((j, i), 0.4, color='grey')
                ax.add_patch(circle)
                # circle = Circle((j, i),0.5, fill = False, edgecolor = 'blue', linewidth = 1)
                # ax.add_patch(circle)
                # ax.text(j, i, str([j,i]), color='black', fontsize=5, ha='center', va='center')
        for k in outfit:
            circle = Circle((k[0], k[1]), 0.4, color='red')
            ax.add_patch(circle)
        # np.save('position_footprint_lid', position)
        # np.save('y_x_lid',[rows, cols])
        for k in outfit_:
            circle = Circle((k[0], k[1]), 0.45, fill=False, edgecolor='blue', linewidth=2.5)
            ax.add_patch(circle)

        # Setting the limits
        ax.set_xlim(-1, cols)
        ax.set_ylim(-1, rows)
        ax.set_aspect('equal', adjustable='box')

        # Hide the axes
        plt.axis('off')

        # Display the plot
        plt.show()

    def find_plane(self, contact_outfit_pose, x0, y0, z0):
        # Convert to a numpy array and separate x, y, z
        contact_outfit_pose_np = np.array(contact_outfit_pose)
        X = contact_outfit_pose_np[:, :2]  # X and Y coordinates
        y = contact_outfit_pose_np[:, 2]  # Z coordinate

        # Apply RANSAC
        ransac = RANSACRegressor()
        ransac.fit(X, y)

        # Coefficients and intercept
        coef = ransac.estimator_.coef_
        intercept = ransac.estimator_.intercept_

        # Plane equation: coef[0]*x + coef[1]*y + z + intercept = 0
        print("Plane equation: {:.2f}x + {:.2f}y + z + {:.2f} = 0".format(coef[0], coef[1], intercept))

        # Plotting
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot original points
        ax.scatter(contact_outfit_pose_np[:, 0], contact_outfit_pose_np[:, 1], contact_outfit_pose_np[:, 2],
                   color='blue', marker='o', label='Original Points')

        # Generate grid to plot the plane
        xx, yy = np.meshgrid(np.arange(X[:, 0].min(), X[:, 0].max()), np.arange(X[:, 1].min(), X[:, 1].max()))
        zz = (-intercept - coef[0] * xx - coef[1] * yy) / 1.0

        # Plot the plane
        # ax.plot_surface(xx, yy, zz, color='red', alpha=0.5)
        ax.quiver(x0, y0, 1, coef[0], coef[1], 1, length=0.03, color='blue')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title('3D Points and Fitted Plane using RANSAC')
        plt.legend()
        plt.show()
        return coef[0], coef[1], intercept

    def detection_start(self):
        rows, cols = self.y_num, self.x_num

        position_cup_drop = []
        position_cup_contact = []
        position_cup_contact_pos = []
        for i in range(rows):
            for j in range(cols):
                num_now = i * self.x_num + j
                if num_now in self.drop_in_id:
                    position_cup_drop.append([j, i])
        outfit_cup = self.outline(position_cup_drop)
        outfit_add_cup = self.outline_add(outfit_cup)
        self.draw_spheres(self.y_num, self.x_num, position_cup_drop, outfit_add_cup)

        print(self.contact_id)

        position_cup_contact_pos = []
        for ii in outfit_add_cup:
            j = ii[0]
            i = ii[1]
            num_now = i * self.x_num + j
            if num_now in self.contact_id:
                index = self.contact_id.index(num_now)
                position_cup_contact_pos.append(self.contact_pos[index])

        self.find_plane(position_cup_contact_pos, self.pos_cup[0], self.pos_cup[1], self.pos_cup[2])



