import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
import time
import pybullet_data

class Match:
    def __init__(self,
                 pos_cup,
                 pos_lid,
                 best_angle,
                 x_rotation = 180):
        self.pos_cup = pos_cup
        self.pos_lid = pos_lid
        self.best_angle = best_angle
        self.x_rotation = x_rotation

    def rotation_matrix_to_euler_angles_rad(self, R):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

        singular = sy < 1e-6  # Check for singularity

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def match_calcuation(self):
        x_rotation_rad = np.radians(self.x_rotation)
        wpa = np.array([[self.pos_lid[0]], [self.pos_lid[1]], [self.pos_lid[2]], [1]])
        rot1 = np.array([[1, 0, 0, 0],
                         [0, np.cos(x_rotation_rad), -np.sin(x_rotation_rad), 0],
                         [0, np.sin(x_rotation_rad), np.cos(x_rotation_rad), 0],
                         [0, 0, 0, 1]])
        best_angle_radian = np.radians(self.best_angle)
        rot2 = np.array([[np.cos(best_angle_radian), np.sin(best_angle_radian), 0, 0],
                         [-np.sin(best_angle_radian), np.cos(best_angle_radian), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        wto = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 1],
                        [0, 0, 0, 1]])
        wto1 = wto @ rot2 @ rot1
        opa = np.linalg.inv(wto) @ wpa
        wpa1 = wto1 @ opa
        wpb = np.array([[self.pos_cup[0]], [self.pos_cup[1]], [self.pos_cup[2]], [1]])
        tran = wpb - wpa1
        rotation_matrix = wto1[:3, :3]
        rotation_quat = R.from_matrix(rotation_matrix).as_quat()
        euler = self.rotation_matrix_to_euler_angles_rad(np.array(rotation_matrix))
        return tran, euler

    def start_match(self):
        tran, euler = self.match_calcuation()
        # Start PyBullet in GUI or DIRECT mode
        physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

        # Set the path for PyBullet to find additional resources
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.resetDebugVisualizerCamera(1, 90 -20, [0.5, 0, 1])

        robotId = p.loadURDF("data/1/cup.urdf",
                             basePosition=np.array([0, 0, 1]),
                             baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                             useFixedBase=True)

        p.setGravity(0, 0, -10)

        obj_curr_aabb = p.getAABB(robotId)
        # Object CoM
        xmin, ymin, zmin = obj_curr_aabb[0]
        xmax, ymax, zmax = obj_curr_aabb[1]
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        center_z = (zmin + zmax) / 2
        center = [center_x, center_y, center_z]
        obj_com_zero = np.array(center)
        obj_com_origin = obj_com_zero - np.array([0, 0, 1])

        # Create constraint on the cup to fix its position
        p.changeDynamics(robotId, -1, mass=1)
        constraint_Id = p.createConstraint(
            robotId,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=obj_com_origin,
            childFramePosition=obj_com_zero)

        # Basic simulation loop
        for x in range(-2, 3):
            for y in range(-2, 3):
                print(x, y)
                lid = p.loadURDF("data/1/lid.urdf",
                                 basePosition=np.array([tran[0] + 0.002 * x, tran[1] + 0.002 * y, tran[2] + 1.2]),
                                 baseOrientation=p.getQuaternionFromEuler([euler[0], euler[1], euler[2]]),
                                 useFixedBase=True)
                zzz = p.getBasePositionAndOrientation(lid)[0][2]

                p.changeDynamics(lid, -1, mass=1)
                time.sleep(1)
                for i in range(1000):
                    p.stepSimulation()
                    if p.getContactPoints(robotId, lid):
                        time.sleep(1)
                        lid_aabb = p.getAABB(lid)
                        cup_aabb = p.getAABB(robotId)
                        print(cup_aabb[1][2] - lid_aabb[0][2])
                        p.removeBody(lid)
                        break
                    time.sleep(1. / 240.)

        # Disconnect from the PyBullet server
        p.disconnect()
