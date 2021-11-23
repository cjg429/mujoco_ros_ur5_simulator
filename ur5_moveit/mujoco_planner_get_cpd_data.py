#!/usr/bin/env python
import os
import sys
import numpy as np
from copy import deepcopy

import rospy
sys.path.append(os.path.join('/home/rllab/catkin_ws/src/'))
#from moveit_tutorials.srv import 
from ur_msgs.srv import JointTrajectory
from moveit_msgs.srv import *
import geometry_msgs

import mujoco_py
from mujoco_py import load_model_from_path, MjSim, MjViewer

import matplotlib.pyplot as plt


kp = 50.
kd = 14.14213562

torques_high = [50., 100., 50., 50., 15., 15.]
torques_low = [-50., -100., -50., -50., -15., -15.]

ARM_JOINT_NAME = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

get_planning_scene_proxy = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
apply_planning_scene_proxy = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)

def joint_position_control(sim, goal_qpos, last_pos=False):
    joint_pos = np.array(sim.data.qpos[0:6])

    position_error = goal_qpos[0:6] - joint_pos

    # calculate torque values
    torques = goal_qpos[0:6] #position_error

    # reach condition check
    position_error = position_error
    error_bound = 0.01 if last_pos else 0.01
    goal_reach = True
    for i in range(len(position_error)):
        if np.abs(position_error[i]) > np.abs(error_bound):
            goal_reach = False
            break
    return torques, goal_reach

def move_to_target(sim, viewer, grasp=0.0, target_qpos=None, target_endpos=None, render=True):
    resp = get_planning_scene_proxy(GetPlanningSceneRequest())
    current_scene = resp.scene

    next_scene = deepcopy(current_scene)
    next_scene.robot_state.joint_state.name = ARM_JOINT_NAME
    next_scene.robot_state.joint_state.position = sim.data.qpos[0:6]
    next_scene.robot_state.joint_state.velocity = [0]*6
    next_scene.robot_state.joint_state.effort = [0]*6
    next_scene.robot_state.is_diff = True

    next_scene.is_diff = True
    req = ApplyPlanningSceneRequest()
    req.scene = next_scene 
    resp = apply_planning_scene_proxy(req)
    for _ in range(100):
        rospy.sleep(0.001)

    # get trajectory from moveit
    getJointTrajectory = rospy.ServiceProxy('send_joint_trajectory', JointTrajectory)
    rospy.wait_for_service('send_joint_trajectory')
    joint_trajectory = getJointTrajectory(ARM_JOINT_NAME, target_qpos, target_endpos)
    for _ in range(100):
        rospy.sleep(0.001)
    
    qpos_trajectory = np.array([p.positions[0:6] for p in joint_trajectory.plan.points])

    total_step = 0
    for i in range(len(qpos_trajectory)):
        #while True:
        while total_step < 500:
            torques, is_reach = joint_position_control(sim, qpos_trajectory[i], i==len(qpos_trajectory)-1)
            sim.data.ctrl[0:6] = torques
            sim.data.ctrl[6:8] = grasp
            #sim.data.ctrl[-1] = -0.98
            sim.step()
            if render:
                viewer.render()
            total_step = total_step + 1
            if is_reach:
                print("reach the {}-th goal pos".format(i+1))
                break

def grasp_w_sensor(sim, viewer, target_qpos=None, render=False):
    if target_qpos is None:
        target_qpos = sim.data.qpos[0:6]

    for i in range(1500):
        sim.data.ctrl[0:6] = target_qpos
        sim.data.ctrl[6:8] = 1.0
        #sim.data.ctrl[-1] = -0.98
        sim.step()
        if render:
            viewer.render()
        if sim.data.sensordata[0] > 200 and sim.data.sensordata[1] > 200:
            return

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def get_camera_pos(arena_pos, t_angle):
    rotation_ = np.array([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]])

    phi = 0
    theta = t_angle[1] #2
    psi = t_angle[2]   #1

    x1 = np.cos(psi) * np.cos(theta) * np.cos(phi) - np.sin(psi) * np.sin(phi)
    x2 = np.cos(psi) * np.sin(phi) + np.cos(theta) * np.cos(phi) * np.sin(psi)
    x3 = -np.cos(phi) * np.sin(theta)
    y1 = -np.cos(phi) * np.sin(psi) - np.cos(psi) * np.cos(theta) * np.sin(phi)
    y2 = np.cos(psi) * np.cos(phi) - np.cos(theta) * np.sin(psi) * np.sin(phi)
    y3 = np.sin(theta) * np.sin(phi)
    z1 = np.cos(psi) * np.sin(theta)
    z2 = np.sin(psi) * np.sin(theta)
    z3 = np.cos(theta)

    drot = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]])
    rotation = rotation_.dot(drot)
    mat = np.array([[0, -1, 0],[-1, 0, 0],[0, 0, -1]])
    rotation = rotation.dot(mat)

    #t_pos = arena_pos + np.array([0.0, 0.0, 0.28]) + 0.4 * np.array([np.sin(phi) * np.cos(theta), -np.sin(phi) *  np.sin(theta), np.cos(phi)])
    t_pos = arena_pos + np.array([0, 0, 0.0]) + 0.6 * np.array([np.sin(theta) * np.cos(psi), -np.sin(theta) * np.sin(psi), np.cos(theta)])

    return t_pos, rotation

def get_arm_rotation(t_angle):
    rotation_ = np.array([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]])

    phi = t_angle[0]
    theta = t_angle[1]
    psi = t_angle[2]

    x1 = np.cos(psi) * np.cos(theta) * np.cos(phi) - np.sin(psi) * np.sin(phi)
    x2 = np.cos(psi) * np.sin(phi) + np.cos(theta) * np.cos(phi) * np.sin(psi)
    x3 = -np.cos(phi) * np.sin(theta)
    y1 = -np.cos(phi) * np.sin(psi) - np.cos(psi) * np.cos(theta) * np.sin(phi)
    y2 = np.cos(psi) * np.cos(phi) - np.cos(theta) * np.sin(psi) * np.sin(phi)
    y3 = np.sin(theta) * np.sin(phi)
    z1 = np.cos(psi) * np.sin(theta)
    z2 = np.sin(psi) * np.sin(theta)
    z3 = np.cos(theta)
    
    drot = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]])
    rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    rotation = rotation.dot(drot)

    return rotation

def get_vertical_image(sim, mjpy_model, camera_pos, vis_on=True):
    camera_id = sim.model.camera_name2id("eye_in_hand")
    camera_obs = sim.render(
        camera_name="eye_in_hand",
        width=256,
        height=256,
        depth=True
    )

    #print(sim.data.cam_xmat[camera_id])
    #pnt_hem, rot_mat = get_camera_pos(camera_pos, np.array([0.0, 0.0, 0.0]))
    pnt_hem = camera_pos
    rot_mat = np.identity(3)
    sim.data.cam_xpos[camera_id] = pnt_hem
    sim.data.cam_xmat[camera_id] = rot_mat.flatten()
    
    camera_obs = sim.render(
        camera_name="eye_in_hand",
        width=256,
        height=256,
        depth=True
    )

    if True:
        vertical_color_image, ddd = camera_obs

    extent = mjpy_model.stat.extent
    near = mjpy_model.vis.map.znear * extent
    far = mjpy_model.vis.map.zfar * extent

    ddd = near / (1 - ddd * (1 - near / far))
    vertical_depth_image = np.where(ddd > 0.25, ddd, 1)

    if vis_on:
        plt.imshow(np.flip(vertical_color_image, axis=0))
        plt.show()
        plt.imshow(np.flip(vertical_depth_image, axis=0))
        plt.show()

    return np.flip(vertical_color_image, axis=0), np.flip(vertical_depth_image, axis=0)

def main():
    # init node
    rospy.init_node('ur5_mujoco', anonymous=True)

    # make MjSim
    from utility import merge_robot_and_object
    save_xml_path = 'models/ur5_w_object.xml'
    tree, num_joint = merge_robot_and_object('models/ur5_w_robotiq.xml', 'models/ball_joint.xml')
    tree.write(open(save_xml_path, 'wb'))
    mjpy_model = load_model_from_path(save_xml_path)
    sim = MjSim(mjpy_model)
    viewer = MjViewer(sim)
    viewer._hide_overlay = True

    #initial_joint_qpos = [1.37787739e+00, -1.11972546e+00, 4.52491864e-01, -9.09825479e-01, -1.56571951e+00, -1.81843747e-01] # initial pos = [0, 0.2, 1.5]
    initial_joint_qpos = [1.29596758, -1.67471806, 1.13343039, -1.03217018, -1.57151726, -0.27534158] # initial pos = [0, 0, 1.5]
    sim.data.qpos[0:6] = initial_joint_qpos
    sim.forward()
    sim.step()
    viewer.render()

    from scipy.spatial.transform import Rotation
    import random
    import imageio
    import pickle
    from sympy import Point, Line, Segment

    ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

    if ros_path in sys.path:

        sys.path.remove(ros_path)

    import cv2

    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

    def get_antipodal_grasps(d_image, num_candidates=5, render=True):
        depth_binary_map = np.zeros(d_image.shape)
        depth_trunc = np.max(d_image) - 0.001
        for u in range(0, d_image.shape[0]):
            for v in range(0, d_image.shape[1]):
                z = d_image[u, v]
                if z < depth_trunc:
                    depth_binary_map[u, v] = 1

        # extracting the contours from the given binary image
        depth_binary_map = (255.0 * depth_binary_map).astype(np.uint8)
        #contours, hierarchy = cv2.findContours(depth_binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(depth_binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        sift_contours = []
        prob_contours = []
        for i in range(len(contours)):
            sift_contours.append(np.squeeze(contours[i]))
            prob_contours.append(len(contours[i]))

        prob_contours = prob_contours / np.sum(prob_contours)
        print("prob conttours: ", prob_contours)
        #sift_contours = np.concatenate(sift_contours, axis=0)
        #sift_contours = np.squeeze(sift_contours)
        
        #line_num = len(sift_contours)

        sampled_grasps = []
        cand = 0
        while cand < num_candidates:
            sample_segment_idx = np.random.choice(len(prob_contours), 1, p=prob_contours)[0]
            #print(sample_segment_idx, len(sift_contours[sample_segment_idx]))
            #sample_point_idx = 0
            #while sample_point_idx != 0 and sample_point_idx != len(sift_contours[sample_segment_idx]) - 1:
            #sample_point_idx = np.random.randint(range(1, len(sift_contours[sample_segment_idx])-1), size=1)[0]
            sample_point_idx = np.random.randint(1, len(sift_contours[sample_segment_idx])-1)
            sample_point = sift_contours[sample_segment_idx][sample_point_idx]
            sample_point_x, sample_point_y = sample_point[0], sample_point[1]
            near_point = sift_contours[sample_segment_idx][sample_point_idx-1:sample_point_idx+2]

            if render:
                plt.scatter(sample_point_x, sample_point_y, color='red')

            line_normal = near_point[2] - near_point[0]
            normal = np.array([line_normal[1], -line_normal[0]])
            next_x = sample_point[0] + int(3 * normal[0])
            next_y = sample_point[1] + int(3 * normal[1])
            if depth_binary_map[next_x, next_y] > 127.5:
                normal = np.array([-line_normal[1], line_normal[0]])
            normal = normal / np.linalg.norm(normal)
            next_x = sample_point[0] + int(100 * normal[0])
            next_y = sample_point[1] + int(100 * normal[1])

            sample_point_x = sample_point_x - int(20 * normal[0])
            sample_point_y = sample_point_y - int(20 * normal[1])

            sampled_grasps.append([[sample_point_x, sample_point_y], [next_x, next_y]])

            if render:
                plt.plot([sample_point_x, next_x], [sample_point_y, next_y], color='black')

            cand = cand + 1
        
        if render:
            plt.imshow(depth_binary_map, cmap='gray')
            plt.show()

        return sampled_grasps

    def get_antipodal_grasps_previous(d_image, num_candidates=5, render=True):
        depth_binary_map = np.zeros(d_image.shape)
        depth_trunc = np.max(d_image) - 0.001
        for u in range(0, d_image.shape[0]):
            for v in range(0, d_image.shape[1]):
                z = d_image[u, v]
                if z < depth_trunc:
                    depth_binary_map[u, v] = 1

        # extracting the contours from the given binary image
        depth_binary_map = (255.0 * depth_binary_map).astype(np.uint8)
        #contours, hierarchy = cv2.findContours(depth_binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(depth_binary_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        print("contour shape: ", len(contours))
        for i in range(len(contours)):
            print("sub contour shape: ", contours[i].shape)
        #plt.imshow(depth_binary_map, cmap='gray')

        sift_contours = []
        for i in range(contours.shape[0]-1):
            pt_1 = contours[i]
            pt_2 = contours[i+1]
            if np.linalg.norm(pt_2 - pt_1) > 50:
                plt.plot([pt_1[0], pt_2[0]], [pt_1[1], pt_2[1]], color='red')
                sift_contours.append([[pt_1[0], pt_2[0]], [pt_1[1], pt_2[1]]])
        #plt.show()
        if render:
            plt.imshow(depth_binary_map, cmap='gray')

        cand = 0
        sampled_grasps = []
        #for cand in range(num_candidates):
        while cand < num_candidates:
            line_num = len(sift_contours)
            line_select_idx = np.random.randint(line_num, size=1)[0]
            weight = np.random.random()
            sample_point_x = weight * sift_contours[line_select_idx][0][0] + (1 - weight) * sift_contours[line_select_idx][0][1]
            sample_point_y = weight * sift_contours[line_select_idx][1][0] + (1 - weight) * sift_contours[line_select_idx][1][1]
            theta = 2 * np.pi * np.random.random()
            sec_x = sample_point_x + 25 * np.cos(theta)
            sec_y = sample_point_y + 25 * np.sin(theta)
            sample_point_x = sample_point_x - 10 * np.cos(theta)
            sample_point_y = sample_point_y - 10 * np.sin(theta)
            sampled_grasps.append([[sample_point_x, sample_point_y], [sec_x, sec_y]])
            cand = cand + 1
            if render:
                plt.scatter(sample_point_x, sample_point_y, color='blue')
                plt.plot([sample_point_x, sec_x], [sample_point_y, sec_y], color='black')

        if render:
            plt.show()
        return sampled_grasps

    camera_pos = [0, 0.3, 1.8]
    data_save_dir = "mask_data"
    import json

    def _pixel2pos  (X, Y, depth, arena_pos=[0.7, -0.25, 0.57]):
        #fovy = 45 #self.sim.model.cam_fovy[0]
        #h, w = 256., 256. 
        #f = 0.5 * h / np.tan(fovy * np.pi / 360.)
        fovy = np.pi / 4.0
        h, w = 256., 256. 
        f = 0.5 * h / np.tan(fovy / 2.0)
        x = (X - w / 2.0) * depth / f 
        y = -(Y - h / 2.0) * depth / f
        return x + arena_pos[0], y + arena_pos[1]

    for _ in range(0, 1):
        print("Go to the initial pose")

        for i in range(500):
            sim.data.ctrl[0:6] = initial_joint_qpos
            sim.step()
        viewer.render()

        for step in range(10000):

            """rot_c_im, rot_d_im = get_vertical_image(sim, mjpy_model, camera_pos, vis_on=False)

            save_rot_d_im = rot_d_im * 1000
            save_rot_d_im = save_rot_d_im.astype('uint16')

            imageio.imwrite(os.path.join(data_save_dir, 'rgb_%04i.jpg'%(step)), rot_c_im)
            imageio.imwrite(os.path.join(data_save_dir, 'd_%04i.png'%(step)), save_rot_d_im)"""

            sim.data.ctrl[8:8+num_joint] = 0 #\np.ones(num_joint) #np.random.rand(num_joint) * 2.0 - 1.0
            sim.step()
            viewer.render()

        """for step in range(1000):
            sim.data.ctrl[8:8+num_joint] = 3.0
            sim.step()
            viewer.render()"""

if __name__ == '__main__':
    main()
