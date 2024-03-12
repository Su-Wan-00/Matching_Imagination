from __future__ import division, print_function

import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math
import trimesh


class Containability(object):
    """
    Class for testing containability of an object
    """
    def __init__(self,
                 obj_urdf,
                 obj_vhacd_mesh,
                 content_urdf,
                 rotate=False,
                 translate=False,
                 friction=True,
                 restitution=True,
                 obj_zero_pos=[0, 0, 1],
                 obj_zero_orn=[0, 0, 0],
                 check_process=False):
        """
        @type  obj_urdf: string
        @param obj_urdf: object urdf file path
        @type  obj_vhacd_mesh: string
        @param obj_vhacd_mesh: object vhacd convex decomposed file path
        @type  content_urdf: string
        @param content_urdf: urdf of the content
        @type  rotate: bool
        @param rotate: whether to include rotation perturbation
        @type  translate: bool
        @param translate: whether to include translation perturbation
        @type  friction: bool
        @param friction: whether to enable friction for spheres
        @type  restitution: bool
        @param restitution: whether to enable restitution for spheres
        @type  obj_zero_pos: list of float / numpy.ndarray
        @param obj_zero_pos: the position of the object w.r.t. its position in the world in the simulation. 
            If set as [0, 0, 0], the position of the object is the same as it in the world frame
        @type  obj_zero_orn: list of float / numpy.ndarray
        @param obj_zero_orn: similar to obj_zero_pos, but for rotation
        @type  check_process: bool
        @param check_process: whether to slow down the simulation to check
        @type  mp4_dir: string
        @param mp4_dir: if set as not None, it will save a mp4 file of the imagination process in the directory
        @type  object_name: string
        @param object_name: name of the object. Note it is not the data name. A data is about a single take of
            the scene. A data can have many object in the scene
        """

        # Hyperparameter
        self.sphere_num_max = 20000
        self.sphere_num_min = 1
        self.sphere_in_percentage_threshold = 0.0
        self.sphere_urdf = content_urdf
        self.sphere_in_percentage = 0.0
        self.sphere_x_range = 0.0
        self.sphere_y_range = 0.0
        self.drop_sphere_num = 0
        self.x_sphere_num = 0
        self.y_sphere_num = 0

        # Restitution
        self.object_restitution = 0.1
        self.plane_restitution = 0.1

        self.obj_urdf = obj_urdf
        self.containability = False

        # Simulation Parameter
        self.simulation_iteration = 1500
        self.check_process = check_process
        self.rotate = rotate  # if add rotate
        self.translate = translate  # if add translate
        self.friction = friction  # if add friction for the content
        self.restitution = restitution

        # Sphere Information
        self.sphere_id = []
        self.sphere_drop_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.sphere_drop_pos = []
        self.sphere_in_drop_pos = []
        self.sphere_drop_z = 0
        self.sphere_restitution = 0.1
        self.sphere_lateralfriction = 0.005
        self.sphere_spinningfriction = 0.5
        self.sphere_rollingfriction = 0.5

        # Set the world
        if not check_process:
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setGravity(0,0,-9.8)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Reset debug camera postion
        # p.resetDebugVisualizerCamera(1.0, 0, -44, [-0.09, -0.1, 1])
        p.resetDebugVisualizerCamera(0.1, 0, -60, [0.5, -0.2, 1.3])
        # p.resetDebugVisualizerCamera(0.2, 270, -10, [0.5, 0, 1.6])

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, restitution=self.plane_restitution)

        # Load object
        self.obj_zero_pos = np.array(obj_zero_pos)
        self.obj_zero_orn = p.getQuaternionFromEuler(
            obj_zero_orn)  # Quaternion
        self.obj_id = p.loadURDF(self.obj_urdf,
                                 basePosition=self.obj_zero_pos,
                                 baseOrientation=self.obj_zero_orn,
                                 useFixedBase=True,
                                 )
        p.changeDynamics(self.obj_id, -1, restitution=self.object_restitution, mass=1)

        # Object CoM
        self.obj_vhacd_mesh = trimesh.load(obj_vhacd_mesh)
        self.obj_com_origin = self.obj_vhacd_mesh.center_mass
        self.obj_com_zero = self.obj_com_origin + self.obj_zero_pos
        self.obj_com_origin1 = [self.obj_com_origin[0], self.obj_com_origin[1], -0.1]

        # Get the bounding box of the cup
        self.obj_curr_aabb = p.getAABB(self.obj_id)

        # Create constraint on the cup to fix its position
        p.changeDynamics(self.obj_id, -1, mass=1)
        self.constraint_Id = p.createConstraint(
            self.obj_id,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            jointAxis=[self.obj_com_zero[0], self.obj_com_zero[1], 0],
            parentFramePosition=self.obj_com_origin,
            childFramePosition=self.obj_com_zero)

    def load_sphere(self, obj_curr_aabb):
        """ 
        Load sphere before simulation. 
        Make sure that pybullet has already been connected before calling this function.

        @type  obj_curr_aabb: tuple of tuple. Refer to the return of p.getAABB()
        @param obj_curr_aabb: object current axis align bounding box (AABB). 
        """

        # Measure the dimension of each content
        sphere = p.loadURDF(self.sphere_urdf, basePosition=[0, 0, -1])
        p.changeDynamics(sphere,
                         -1,
                         restitution=self.sphere_restitution,
                         lateralFriction=self.sphere_lateralfriction,
                         spinningFriction=self.sphere_spinningfriction,
                         rollingFriction=self.sphere_rollingfriction)

        sphere_aabb = p.getAABB(sphere)
        p.removeBody(sphere)

        self.sphere_x_range = sphere_aabb[1][0] - sphere_aabb[0][0]
        self.sphere_y_range = sphere_aabb[1][1] - sphere_aabb[0][1]

        obj_aabb_xy_center = [(obj_curr_aabb[0][i] + obj_curr_aabb[1][i]) / 2
                              for i in range(2)]
        self.obj_aabb_xy_center = obj_aabb_xy_center
        obj_aabb_xy_range = [
            abs(obj_curr_aabb[0][i] - obj_curr_aabb[1][i]) for i in range(2)
        ]

        # Dropping from 1cm above the object
        self.sphere_drop_z = [obj_curr_aabb[1][2] + 0.01] # change 0.01 to 0.1
        self.z_sphere_num = 1

        self.sphere_dis_x = self.sphere_x_range
        self.x_sphere_num = int(obj_aabb_xy_range[0] / self.sphere_dis_x)

        self.sphere_dis_y = self.sphere_y_range
        self.y_sphere_num = int(obj_aabb_xy_range[1] / self.sphere_dis_y)

        # print(self.x_sphere_num)
        # print(self.y_sphere_num)

        self.drop_sphere_num = self.x_sphere_num * self.y_sphere_num * self.z_sphere_num

        # Too many contents for dropping
        if self.drop_sphere_num > self.sphere_num_max:
            self.sphere_dist_scale = math.sqrt(self.drop_sphere_num /
                                               self.sphere_num_max)
            self.x_sphere_num = int(
                obj_aabb_xy_range[0] /
                (self.sphere_dis_x * self.sphere_dist_scale))
            self.y_sphere_num = int(
                obj_aabb_xy_range[1] /
                (self.sphere_dis_y * self.sphere_dist_scale))
            self.drop_sphere_num = self.x_sphere_num * self.y_sphere_num * self.z_sphere_num
        # Too few contents for dropping
        elif self.drop_sphere_num < self.sphere_num_min:
            self.z_sphere_num = int(
                self.sphere_num_min / self.drop_sphere_num) + 1
            for i in range(1, self.z_sphere_num):
                self.sphere_drop_z.append(self.sphere_drop_z[0] + i * 0.05)
            self.drop_sphere_num = self.x_sphere_num * self.y_sphere_num * self.z_sphere_num

        # Set up sphere
        for i in range(self.drop_sphere_num):
            sphere = p.loadURDF(self.sphere_urdf)
            if (self.friction) and (self.restitution):
                # add both friction and restitution
                p.changeDynamics(sphere,
                                 -1,
                                 restitution=self.sphere_restitution,
                                 lateralFriction=self.sphere_lateralfriction,
                                 spinningFriction=self.sphere_spinningfriction,
                                 rollingFriction=self.sphere_rollingfriction,
                                 mass=0.01)
            elif (self.friction) and (not self.restitution):
                # add friction but not restitution
                p.changeDynamics(sphere,
                                 -1,
                                 restitution=0,
                                 lateralFriction=self.sphere_lateralfriction,
                                 spinningFriction=self.sphere_spinningfriction,
                                 rollingFriction=self.sphere_rollingfriction,
                                 )
            elif (not self.friction) and (self.restitution):
                # add resitution but not friction
                p.changeDynamics(sphere,
                                 -1,
                                 restitution=self.sphere_restitution,
                                 lateralFriction=0,
                                 spinningFriction=0,
                                 rollingFriction=0,
                                 )
            else:
                # add not restitution or friction
                p.changeDynamics(sphere,
                                 -1,
                                 restitution=0,
                                 lateralFriction=0,
                                 spinningFriction=0,
                                 rollingFriction=0,
                                 )

            self.sphere_id.append(sphere)

        sphere_drop_x = np.linspace(-obj_aabb_xy_range[0] / 2,
                                    obj_aabb_xy_range[0] / 2,
                                    self.x_sphere_num) + obj_aabb_xy_center[0]
        sphere_drop_y = np.linspace(-obj_aabb_xy_range[1] / 2,
                                    obj_aabb_xy_range[1] / 2,
                                    self.y_sphere_num) + obj_aabb_xy_center[1]

        # Reset the sphere_drop_pos
        self.sphere_drop_pos = []

        # Reset sphere position
        for i in range(self.drop_sphere_num):
            z_idx = int(i / (self.x_sphere_num * self.y_sphere_num))
            y_idx = int((i - z_idx * self.x_sphere_num * self.y_sphere_num) /
                        self.x_sphere_num)
            x_idx = (i - z_idx * self.x_sphere_num *
                     self.y_sphere_num) % self.x_sphere_num

            sphere_start_pos = (sphere_drop_x[x_idx], sphere_drop_y[y_idx],
                                self.sphere_drop_z[z_idx])

            # print(z_idx,y_idx,x_idx)
            # print(sphere_start_pos)

            self.sphere_drop_pos.append(sphere_start_pos)
            p.resetBasePositionAndOrientation(self.sphere_id[i],
                                              posObj=sphere_start_pos,
                                              ornObj=self.sphere_drop_orn)

    def check_in_cup_sphere(self, obj_curr_aabb):
        """ 
        Get how many spheres are in the cup and the original position of the sphere
        Also, the drop pos of the in sphere will be updated to self.sphere_in_drop_pos
        
        @type  obj_curr_aabb: tuple of tuple. Refer to the return of p.getAABB()
        @param obj_curr_aabb: object current axis align bounding box (AABB). 
        """

        # Clear the sphere_in_drop_pos memory
        self.sphere_in_drop_pos = []
        self.sphere_in_id = []  # add for visualizing the footprint
        self.sphere_pos_base = []

        # The center and range of the aabb. All scales are magnified by 100 times.
        obj_aabb_center = np.array([
            (obj_curr_aabb[0][i] + obj_curr_aabb[1][i]) / 2 for i in range(3)
        ]) * 100
        obj_aabb_half_range = np.array(
            [abs(obj_curr_aabb[0][i] - obj_curr_aabb[1][i])
             for i in range(3)]) * 100 / 2

        for i in range(self.drop_sphere_num):
            sphere_pos, _ = p.getBasePositionAndOrientation(self.sphere_id[i])
            sphere_pos = np.array(sphere_pos) * 100

            distance_to_center = np.absolute(sphere_pos - obj_aabb_center)
            # in_box = distance_to_center < obj_aabb_half_range
            in_box = sphere_pos[2]>self.obj_curr_aabb[0][2]-0.016#-(self.cylinder_aabb[1][2]-self.cylinder_aabb[0][2])

            if np.all(in_box):
                self.sphere_in_drop_pos.append(np.copy(
                    self.sphere_drop_pos[i]))
                self.sphere_in_id.append(i)
                self.sphere_pos_base.append(sphere_pos)

        # print(self.sphere_in_drop_pos)
        # print(self.sphere_in_id)

        return len(self.sphere_in_drop_pos)

    def get_containability(self):
        """ 
        Get the containabiliyt of the object by dropping sphere onto the object
        and count how many are remained after dropping and perturbation.
        """
        # Load sphere
        self.load_sphere(self.obj_curr_aabb)


        ########################### Drop Sphere Into ############################
        # force = 1

        # Pivot for rotating the object
        pivot = self.obj_com_zero

        if (self.rotate) and (self.translate):
            print(
                "Both rotation and translation perturbations are activated...")
            for i in range(self.simulation_iteration):

                p.stepSimulation()

                if self.check_process:
                    time.sleep(1. / 240.)

                # Rotation for rotation perturbation
                if i > int(1 * self.simulation_iteration / 10) and i <= int(
                        5 * self.simulation_iteration / 10):
                    orn = p.getQuaternionFromEuler([
                        math.pi / 60 * math.sin(
                            math.pi * 2 *
                            (i - int(1 * self.simulation_iteration / 10)) /
                            int(4 * self.simulation_iteration / 10)), 0, 0
                    ])
                    p.changeConstraint(self.constraint_Id,
                                       pivot,
                                       jointChildFrameOrientation=orn,
                                       maxForce=50)
                elif i > int(5 * self.simulation_iteration / 10) and i <= int(
                        9 * self.simulation_iteration / 10):
                    orn = p.getQuaternionFromEuler([
                        0, math.pi / 60 *
                        math.sin(math.pi * 2 *
                                 (i - int(5 * self.simulation_iteration / 10))
                                 / int(4 * self.simulation_iteration / 10)), 0
                    ])
                    p.changeConstraint(self.constraint_Id,
                                       pivot,
                                       jointChildFrameOrientation=orn,
                                       maxForce=50)

                # Horizontal acceleration for translation perturbation
                if i > int(9 * self.simulation_iteration / 10) and i <= int(
                        9.25 * self.simulation_iteration / 10):
                    p.setGravity(0.5, 0.0, -9.81)
                elif i > int(
                        9.25 * self.simulation_iteration / 10) and i <= int(
                            9.5 * self.simulation_iteration / 10):
                    p.setGravity(-0.5, 0.0, -9.81)
                elif i > int(
                        9.5 * self.simulation_iteration / 10) and i <= int(
                            9.75 * self.simulation_iteration / 10):
                    p.setGravity(0.0, 0.5, -9.81)
                elif i > int(
                        9.75 * self.simulation_iteration / 10) and i <= int(
                            10 * self.simulation_iteration / 10):
                    p.setGravity(0.0, -0.5, -9.81)

        elif (not self.rotate) and (self.translate):
            print("Only translation perturbation is activated...")
            for i in range(self.simulation_iteration):
                p.stepSimulation()

                if self.check_process:
                    time.sleep(1. / 240.)

                # Horizontal acceleration for translation perturbation
                if i > int(9 * self.simulation_iteration / 10) and i <= int(
                        9.25 * self.simulation_iteration / 10):
                    p.setGravity(0.5, 0.0, -9.81)
                elif i > int(
                        9.25 * self.simulation_iteration / 10) and i <= int(
                            9.5 * self.simulation_iteration / 10):
                    p.setGravity(-0.5, 0.0, -9.81)
                elif i > int(
                        9.5 * self.simulation_iteration / 10) and i <= int(
                            9.75 * self.simulation_iteration / 10):
                    p.setGravity(0.0, 0.5, -9.81)
                elif i > int(
                        9.75 * self.simulation_iteration / 10) and i <= int(
                            10 * self.simulation_iteration / 10):
                    p.setGravity(0.0, -0.5, -9.81)

        elif (self.rotate) and (not self.translate):
            print("Only rotation perturbation is activated...")
            for i in range(self.simulation_iteration):
                p.stepSimulation()

                if self.check_process:
                    time.sleep(1. / 240.)

                # 2.0: Shake Objects
                if i > int(5 * self.simulation_iteration / 10) and i <= int(
                        7 * self.simulation_iteration / 10):
                    orn = p.getQuaternionFromEuler([
                        math.pi / 60 * math.sin(
                            math.pi * 2 *
                            (i - int(1 * self.simulation_iteration / 10)) /
                            int(4 * self.simulation_iteration / 10)), 0, 0
                    ])
                    p.changeConstraint(self.constraint_Id,
                                       pivot,
                                       jointChildFrameOrientation=orn,
                                       maxForce=50)
                elif i > int(7 * self.simulation_iteration / 10) and i <= int(
                        9 * self.simulation_iteration / 10):
                    orn = p.getQuaternionFromEuler([
                        0, math.pi / 60 *
                        math.sin(math.pi * 2 *
                                 (i - int(5 * self.simulation_iteration / 10))
                                 / int(4 * self.simulation_iteration / 10)), 0
                    ])
                    p.changeConstraint(self.constraint_Id,
                                       pivot,
                                       jointChildFrameOrientation=orn,
                                       maxForce=50)

        elif (not self.rotate) and (not self.translate):
            print("No perturbations are activated...")
            self.sphere_pos_100 = []
            for i in range(self.simulation_iteration):
                p.stepSimulation()
                # for j in range(self.x_sphere_num*self.y_sphere_num):
                #     tem, _ = p.getBasePositionAndOrientation(self.sphere_id[j])
                #     pos[j].append(tem)

                if self.check_process:
                    time.sleep(1. / 240.)
                # if i == 70:
                #     print(i)
                #     for j in range(self.drop_sphere_num):
                #         sphere_pos_100, _ = p.getBasePositionAndOrientation(self.sphere_id[j])
                #         self.sphere_pos_100.append(sphere_pos_100)
                # print(i)
        # p.stopStateLogging(self.state_logging_id)

        ########## Checking sphere ##########
        # Check the x, y, z coordinate of the sphere w.r.t to the x, y, z coordinate of the cup
        sphere_in_box_num = self.check_in_cup_sphere(self.obj_curr_aabb)

        # Calculate how many percentage of the spheres are in the cup
        sphere_num_percentage = sphere_in_box_num / self.drop_sphere_num

        if sphere_num_percentage > self.sphere_in_percentage_threshold:
            print("#####################################")
            # print(self.object_name)
            print(
                "THIS IS A CONTAINER! The sphere in percentage is: {}".format(
                    sphere_num_percentage))
            print("#####################################")
            self.containability = True

        else:
            print("/////////////////////////////////////")
            # print(self.object_name)
            print("THIS IS NOT A CONTAINER!The sphere in percentage is: {}".
                  format(sphere_num_percentage))
            print("/////////////////////////////////////")
            self.containability = False

        return self.containability, sphere_num_percentage

    def find_drop_center(self):
        """ 
        Find the center of the sphere remained after the dropping
        """
        if len(self.sphere_in_drop_pos) < 1:
            print("No sphere remains in the object after drops!")
            return None
        else:
            self.sphere_in_drop_pos = np.array(self.sphere_in_drop_pos)
            x = np.mean(self.sphere_in_drop_pos[:, 0])
            y = np.mean(self.sphere_in_drop_pos[:, 1])
            z = self.sphere_drop_z[0]
            drop_center_curr_frame = np.array([x, y, z])

            # Original frame 2 current frame
            R = np.reshape(
                np.array(p.getMatrixFromQuaternion(self.obj_zero_orn)), (3, 3))
            t = self.obj_zero_pos

            drop_center_original_frame = np.dot(np.linalg.inv(R),
                                                (drop_center_curr_frame - t).T)

            return drop_center_original_frame

    def find_lid_center(self):
        num = len(self.sphere_pos_base)
        x = 0
        y = 0
        for i in range(num):
            x = x + self.sphere_pos_base[i][0]
            y = y + self.sphere_pos_base[i][1]
        x = x / num
        y = y / num
        return [x, y, self.contact_pos[0][2]]

    def visualize_footprint(self, sphere_urdf, marker_sphere_urdf):
        """ 
        Visualize the footprint of an object 
        """

        # Remove all the current sphere
        for sphere_id in self.sphere_id:
            p.removeBody(sphere_id)

        # load sphere
        for idx, pos in enumerate(self.sphere_drop_pos):
            if idx in self.sphere_in_id:
                p.loadURDF(marker_sphere_urdf, basePosition=pos)
            else:
                p.loadURDF(sphere_urdf, basePosition=pos)

        import ipdb
        ipdb.set_trace()

    def disconnect_p(self):
        """
        Disconnet the connection to pybullet
        """
        p.disconnect()

    def find_contact(self):
        self.contact_pos = []
        self.contact_id = []
        for i in range(self.drop_sphere_num):
            contact_info = p.getContactPoints(bodyA=self.obj_id, bodyB=self.sphere_id[i])
            if contact_info:
                self.contact_pos.append(contact_info[0][5])
                self.contact_id.append(i)
        return self.x_sphere_num, self.y_sphere_num, self.contact_pos, self.contact_id
        # contact_info = p.getContactPoints(bodyA=self.obj_id, bodyB=self.sphere_id[42])
        # for i in range(len(contact_info)):
        #     print(contact_info[i][5])
        # print(p.getBasePositionAndOrientation(self.sphere_id[42]))