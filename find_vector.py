import numpy as np
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from sklearn.linear_model import RANSACRegressor
from mpl_toolkits.mplot3d import Axes3D


class Find:
    def __init__(self, x_num, y_num, contact_pos, contact_id, drop_in_pos, drop_in_id):
        self.x_num = x_num # 18
        self.y_num = y_num # 11
        self.num = x_num * y_num
        self.contact_pos = contact_pos
        self.contact_id = contact_id
        self.drop_in_pos = drop_in_pos
        self.drop_in_id = drop_in_id

    def draw_spheres(self):
        # Set up the figure
        fig, ax = plt.subplots()

        # Size of the grid
        rows, cols = self.y_num, self.x_num
        position = []

        # Drawing the circles
        for i in range(rows): # 11
            for j in range(cols): # 18
                num_now = i * self.x_num + j
                if (num_now in self.contact_id or num_now in self.drop_in_id):
                    position.append([j, i])
                if (num_now in self.contact_id and num_now in self.drop_in_id):
                    # if self.contact_pos[self.contact_id.index(num_now)][2] > 1:
                    circle = Circle((j, i), 0.4, color='red')
                    ax.add_patch(circle)
                elif num_now in self.contact_id:
                    circle = Circle((j, i), 0.4, color='red')
                    ax.add_patch(circle)
                elif num_now in self.drop_in_id:
                    circle = Circle((j, i), 0.4, color='red')
                    ax.add_patch(circle)
                    # else:
                    #     circle = Circle((j, i), 0.4, color='red')
                    #     ax.add_patch(circle)
                else:
                    circle = Circle((j, i), 0.4, color='grey')
                    ax.add_patch(circle)
                # ax.text(j, i, str(num_now), color='black', fontsize=8, ha='center', va='center')
        self.position_footprint = position
        # np.save('y_x_lid',[rows, cols])

        # Setting the limits
        ax.set_xlim(-1, cols)
        ax.set_ylim(-1, rows)
        ax.set_aspect('equal', adjustable='box')

        # Hide the axes
        plt.axis('off')

        # Display the plot
        plt.show()

    def pick_center(self):
        id = sorted(set(self.contact_id) | set(self.drop_in_id))
        x = 0
        y = 0
        for i in id:
            if i in self.drop_in_id:
                x = x + self.drop_in_pos[self.drop_in_id.index(i)][0]
                y = y + self.drop_in_pos[self.drop_in_id.index(i)][1]
            else:
                x = x + self.drop_in_pos[self.contact_id.index(i)][0]
                y = y + self.contact_pos[self.contact_id.index(i)][1]
        x = x/len(id)
        y = y/len(id)
        return [x,y,self.drop_in_pos[0][2]]

    def pick_center_drop_in(self):
        x = 0
        y = 0
        for i in self.drop_in_id:
            x = x + self.drop_in_pos[self.drop_in_id.index(i)][0]
            y = y + self.drop_in_pos[self.drop_in_id.index(i)][1]
        x = x / len(self.drop_in_id)
        y = y / len(self.drop_in_id)
        return [x, y, self.drop_in_pos[0][2]]

    def pick_outfit(self):
        rows, cols = self.y_num, self.x_num
        contact_outfit1 = []
        contact_outfit1_pos = []
        for i in range(rows): # 11
            for j in range(cols): # 18
                num_now = i * self.x_num + j
                if num_now in self.contact_id:
                    # the first row contains the contact
                    if self.contact_id.index(num_now)==0:
                        for m in range(j,cols):
                            num_now_1 = i * self.x_num + m
                            if num_now_1 in self.contact_id:
                                contact_outfit1.append(num_now_1)
                                contact_outfit1_pos.append(self.contact_pos[self.contact_id.index(num_now_1)])
                        break
                    elif all(m not in self.contact_id for m in range((i+1)*self.x_num,self.num)):
                        for m in range(j,cols):
                            num_now_1 = i * self.x_num + m
                            if num_now_1 in self.contact_id:
                                contact_outfit1.append(num_now_1)
                                contact_outfit1_pos.append(self.contact_pos[self.contact_id.index(num_now_1)])
                        break
                    elif (j == 0 or all(i * self.x_num + m not in self.contact_id for m in range(j))):
                        contact_outfit1.append(num_now)
                        contact_outfit1_pos.append(self.contact_pos[self.contact_id.index(num_now)])
                    elif (j == cols-1 or all(i * self.x_num + m not in self.contact_id for m in range(j+1,cols))):
                        contact_outfit1.append(num_now)
                        contact_outfit1_pos.append(self.contact_pos[self.contact_id.index(num_now)])
        print (contact_outfit1)
        print (contact_outfit1_pos)
        return contact_outfit1, contact_outfit1_pos

    def find_plane(self):

        _,contact_outfit_pose = self.pick_outfit()

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

        # # Plotting
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        #
        # # Plot original points
        # ax.scatter(contact_outfit_pose_np[:, 0], contact_outfit_pose_np[:, 1], contact_outfit_pose_np[:, 2], color='blue', marker='o', label='Original Points')
        #
        # # Generate grid to plot the plane
        # xx, yy = np.meshgrid(np.arange(X[:, 0].min(), X[:, 0].max()), np.arange(X[:, 1].min(), X[:, 1].max()))
        # zz = (-intercept - coef[0] * xx - coef[1] * yy) / 1.0
        #
        # # Plot the plane
        # ax.plot_surface(xx, yy, zz, color='yellow', alpha=0.5)
        #
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # plt.title('3D Points and Fitted Plane using RANSAC')
        # plt.legend()
        # plt.show()
        return coef[0], coef[1], intercept

    def fit_shape(self):
        # kick away bad points
        a, b, d = self.find_plane()
        c = 1
        self.edge_id = []
        self.edge_pos = []
        distance = []
        self.edge_dis = []
        for i in range(len(self.contact_id)):
            distance.append(abs(a * self.contact_pos[i][0] + b * self.contact_pos[i][1] + c * self.contact_pos[i][2] - d) / math.sqrt(a ** 2 + b ** 2 + c ** 2))
        threshold = max(distance)/2.0*1.5
        for i in range(len(self.contact_id)):
            if distance[i] < threshold:
                self.edge_id.append(self.contact_id[i])
                self.edge_pos.append(self.contact_pos[i])
                self.edge_dis.append(distance[i])
        # print(self.edge_id)
        # print(self.edge_pos)

        # project onto plane
        norm = np.sqrt(a ** 2 + b ** 2 + c ** 2)
        len_edge = len(self.edge_dis)
        self.edge_pos_rev = []
        for i in range(len_edge):
            rev = [self.contact_pos[i][0] - self.edge_dis[i] * (a / norm),
                   self.contact_pos[i][1] - self.edge_dis[i] * (b / norm),
                   self.contact_pos[i][2] - self.edge_dis[i] * (c / norm)]
            self.edge_pos_rev.append(rev)

        # fit circle
        # Normal vector of the plane
        normal_vector = np.array([a, b, c])

        # Create one vector in the plane
        if not np.isclose(normal_vector[0], 0) or not np.isclose(normal_vector[1], 0):
            in_plane_vector = np.cross(normal_vector, [0, 0, 1])
        else:
            in_plane_vector = np.cross(normal_vector, [0, 1, 0])

        # Create another vector in the plane, orthogonal to the first
        second_in_plane_vector = np.cross(normal_vector, in_plane_vector)

        # Normalize the vectors
        x_axis = in_plane_vector / np.linalg.norm(in_plane_vector)
        y_axis = second_in_plane_vector / np.linalg.norm(second_in_plane_vector)

        # Example points on the plane
        pos = np.array([...])  # replace with your array of 3D points

        # Choose an arbitrary point on the plane as the origin for the 2D coordinate system
        point_on_plane = pos[0]

        # Transform the points
        points_2d = []
        for point in pos:
            relative_point = point - point_on_plane
            x_coord = np.dot(relative_point, x_axis)
            y_coord = np.dot(relative_point, y_axis)
            points_2d.append([x_coord, y_coord])

        points_2d = np.array(points_2d)

























