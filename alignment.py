import numpy as np
import math

import matplotlib.pyplot as plt
from shapely.geometry import Polygon

import math

class Alignment:
    def __init__(self,
                 position_cup,
                 position_lid):
        self.position_cup = position_cup
        self.position_lid = position_lid

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

    def sequence_points(self, points, threshold):
        """Reorder points based on proximity, starting with the first point. Discard points further than 'threshold'."""
        A = points.copy()
        B = []

        while len(A) > 0:
            if len(B) == 0:
                # Start with the first point in A for the first iteration
                point = A[0]
            else:
                # Find the closest point to the last added point in B
                _, point = self.find_closest_point(B[-1], A, threshold)
                if point is None:
                    break  # Exit if no close enough point is found

            # Add the closest point to B and remove it from A
            B.append(point)
            A = np.array([p for p in A if not np.array_equal(p, point)])

        return np.array(B)

    def find_closest_point(self, point, points, threshold):
        """Find the closest point to 'point' within 'points'. If the closest distance is larger than 'threshold', return None."""
        distances = np.sqrt(((points - point) ** 2).sum(axis=1))
        closest_index = np.argmin(distances)
        closest_distance = distances[closest_index]
        if closest_distance > threshold:
            return None, None
        return closest_index, points[closest_index]

    def rotate_point(self, px, py, cx, cy, angle_deg):
        # Convert angle from degrees to radians
        angle_rad = math.radians(angle_deg)

        # Calculate the rotated coordinates
        qx = cx + math.cos(angle_rad) * (px - cx) - math.sin(angle_rad) * (py - cy)
        qy = cy + math.sin(angle_rad) * (px - cx) + math.cos(angle_rad) * (py - cy)
        return qx, qy

    def plot(self, sequenced_points_cup_1, sequenced_points_lid, center_lid):
        x_lid, y_lid = sequenced_points_lid[:, 0], sequenced_points_lid[:, 1]
        plt.plot(x_lid, y_lid, '-o', color='black',
                 label='Outline')  # '-o' creates lines connecting the points, with circles at the points
        plt.fill(x_lid, y_lid, color='red', alpha=0.4)  # 'alpha' controls the transparency
        plt.plot(center_lid[0], center_lid[1], '*', color='black')

        x_cup = [point[0] for point in sequenced_points_cup_1]
        y_cup = [point[1] for point in sequenced_points_cup_1]
        plt.plot(x_cup, y_cup, '-o', color='black',
                 label='Outline')  # '-o' creates lines connecting the points, with circles at the points
        plt.fill(x_cup, y_cup, color='#1E90FF', alpha=0.45)  # 'alpha' controls the transparency

        plt.grid(True)
        plt.axis('equal')  # Ensures the aspect ratio is equal, so the shape isn't distorted
        ax = plt.gca()  # Get current axis

        # Set grid interval to 1
        ax.set_xticks(np.arange(-2, 20, 2))
        ax.set_yticks(np.arange(-2, 14, 2))

        # Hide specific tick labels (1, 2, 3) by setting them to an empty string
        # For other labels, use the original value
        ax.set_xticklabels([])
        ax.set_yticklabels([])

        plt.show()


    def alignment_start(self):
        ##################### cup
        outfit_cup = self.outline(self.position_cup)

        ########## sequence
        points_cup = np.array(outfit_cup)
        threshold = 5.0  # Maximum distance to consider for proximity

        # Sequence the points based on proximity
        sequenced_points_cup = self.sequence_points(points_cup, threshold)
        new_cup = sequenced_points_cup[0]
        sequenced_points_cup = np.vstack((sequenced_points_cup, new_cup))
        # print(len(outfit_cup))
        # print(len(sequenced_points_cup))
        center_cup = points_cup.mean(axis=0)
        # print(center_cup)

        ##################### lid
        outfit_lid = self.outline(self.position_lid)

        ########## sequence
        points_lid = np.array(outfit_lid)
        threshold = 5.0  # Maximum distance to consider for proximity

        # Sequence the points based on proximity
        sequenced_points_lid = self.sequence_points(points_lid, threshold)
        new_lid = sequenced_points_lid[0]
        sequenced_points_lid = np.vstack((sequenced_points_lid, new_lid))
        # print(len(outfit_lid))
        # print(len(sequenced_points_lid))
        center_lid = points_lid.mean(axis=0)
        # print(center_lid)

        ################ overlap
        delta_x = center_lid[0] - center_cup[0]
        delta_y = center_lid[1] - center_cup[1]
        sequenced_points_cup_1 = []
        for i in sequenced_points_cup:
            a = i[0] + delta_x
            b = i[1] + delta_y
            sequenced_points_cup_1.append([a, b])

        self.plot(sequenced_points_cup_1, sequenced_points_lid, center_lid)

        best_angle = 0
        best_IOU = 0
        for i in range(12):
            rotated_cup = [self.rotate_point(x, y, center_lid[0], center_lid[1], 30 * i) for x, y in sequenced_points_cup_1]
            polygon_lid = Polygon(sequenced_points_lid)
            polygon_cup = Polygon(rotated_cup)
            union_area = polygon_cup.union(polygon_lid).area
            intersection_area = polygon_cup.intersection(polygon_lid).area
            IOU = intersection_area / union_area
            if IOU > best_IOU:
                best_IOU = IOU
                best_angle = 30 * i

        angle = best_angle
        for i in range(angle - 30, angle + 31, 2):
            rotated_cup = [self.rotate_point(x, y, center_lid[0], center_lid[1], i) for x, y in sequenced_points_cup_1]
            polygon_lid = Polygon(sequenced_points_lid)
            polygon_cup = Polygon(rotated_cup)
            union_area = polygon_cup.union(polygon_lid).area
            intersection_area = polygon_cup.intersection(polygon_lid).area
            IOU = intersection_area / union_area
            if IOU > best_IOU:
                best_IOU = IOU
                best_angle = i
        print(best_angle)
        self.best_angle = best_angle
        print(best_IOU)

        rotated_cup = [self.rotate_point(x, y, center_lid[0], center_lid[1], best_angle) for x, y in sequenced_points_cup_1]
        rotated_points_cup = []
        for i in rotated_cup:
            rotated_points_cup.append([i[0], i[1]])

        self.plot(rotated_points_cup, sequenced_points_lid, center_lid)