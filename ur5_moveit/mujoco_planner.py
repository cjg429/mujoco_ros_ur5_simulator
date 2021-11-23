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

def move_to_target(sim, viewer, grasp=0.0, target_qpos=None, target_endpos=None):
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
    for i in range(len(qpos_trajectory)):
        while True:
            torques, is_reach = joint_position_control(sim, qpos_trajectory[i], i==len(qpos_trajectory)-1)
            sim.data.ctrl[0:6] = torques
            sim.data.ctrl[6:8] = grasp
            sim.step()
            viewer.render()
            if is_reach:
                print("reach the {}-th goal pos".format(i+1))
                break

def grasp_w_sensor(sim, viewer, target_qpos=None):
    if target_qpos is None:
        target_qpos = sim.data.qpos[0:6]

    for i in range(50000):
        sim.data.ctrl[0:6] = target_qpos
        sim.data.ctrl[6:8] = 1.0
        sim.step()
        viewer.render()
        if sim.data.sensordata[0] > 100 and sim.data.sensordata[1] > 100:
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

"""def gqcnn(self, arm='right', vis_on=False, num_candidates=10):
    di = super()._get_observation()

    if arm == 'right':
        camera_obs = self.sim.render(
            camera_name="eye_on_right_wrist",
            width=self.camera_width,
            height=self.camera_height,
            depth=self.camera_depth
        )
    elif arm == 'left':
        camera_obs = self.sim.render(
            camera_name="eye_on_left_wrist",
            width=self.camera_width,
            height=self.camera_height,
            depth=self.camera_depth
        )

    if self.camera_depth:
        di["image"], ddd = camera_obs
    else:
        di["image"] = camera_obs

    '''while np.min(ddd) > 0.8:
        print("ddd failed")
        camera_obs = self.sim.render(
            camera_name=self.camera_name,
            width=self.camera_width,
            height=self.camera_height,
            depth=self.camera_depth
        )
        di["image"], ddd = camera_obs'''

    extent = self.mjpy_model.stat.extent
    near = self.mjpy_model.vis.map.znear * extent
    far = self.mjpy_model.vis.map.zfar * extent

    di["depth"] = near / (1 - ddd * (1 - near / far))
    di["depth"] = np.where(di["depth"] > 0.25, di["depth"], 1)

    return policy.evaluate_gqcnn(np.flip(di["image"], axis=0), np.flip(di["depth"], axis=0), vis_on=vis_on, num_candidates=num_candidates), np.flip(di["image"], axis=0), np.flip(di["depth"], axis=0)"""


def main():
    # init node
    rospy.init_node('ur5_mujoco', anonymous=True)

    # add ant model to world
    xml_path = os.path.join('ur5_w_robotiq.xml')
    save_xml_path = os.path.join('ur5_w_ant.xml')

    #from model_changer_complicate import changer
    from model_changer_2d import changer
    tree, num_track, graph = changer(xml_path)
    tree.write(open(save_xml_path, 'wb'))

    # make MjSim
    #mjpy_model = load_model_from_path('ur5_w_robotiq.xml')
    mjpy_model = load_model_from_path('ur5_w_ant.xml')
    sim = MjSim(mjpy_model)
    viewer = MjViewer(sim)
    viewer._hide_overlay = True

    """TARGET_JOINT_QPOS = [-4.52900676522404e-05, -1.5707255042219068, -9.767265589907765e-05, -1.5706097928549163, 4.973337803967298e-05, -7.33426604885608e-05]
    move_to_target(sim, viewer, grasp=0.0, target_qpos=TARGET_JOINT_QPOS)
    for _ in range(1000):
        rospy.sleep(0.001)
        viewer.render()"""

    #initial_joint_qpos = [1.37772115e+00, -1.30451888e+00, 1.08151201e+00, -1.35691147e+00, -1.56391774e+00, 1.38117380e+00]
    initial_joint_qpos = [1.37787739e+00, -1.11972546e+00, 4.52491864e-01, -9.09825479e-01, -1.56571951e+00, -1.81843747e-01]
    sim.data.qpos[0:6] = initial_joint_qpos
    sim.forward()
    sim.step()
    viewer.render()

    """TARGET_JOINT_QPOS = [1.37772115e+00, -1.30451888e+00, 1.08151201e+00, -1.35691147e+00, -1.56391774e+00, 1.38117380e+00]
    move_to_target(sim, viewer, grasp=0.0, target_qpos=TARGET_JOINT_QPOS)
    for _ in range(1000):
        rospy.sleep(0.001)
        viewer.render()"""

    """TARGET_JOINT_QPOS = [-4.52900676522404e-05, -1.5707255042219068, -1.57, -1.5706097928549163, 4.973337803967298e-05, -7.33426604885608e-05]
    move_to_target(sim, viewer, target_qpos=TARGET_JOINT_QPOS)
    for _ in range(1000):
        rospy.sleep(0.001)
        viewer.render()"""

    for i in range(10000):
        sim.data.ctrl[0:6] = initial_joint_qpos
        #sim.data.ctrl[-1] = -0.98
        sim.step()
        viewer.render()

    """for i in range(0, 10):
        for j in range(0, 3):
            get_vertical_image(sim, camera_pos=[0, 0.4-0.1*j, 1.5-0.1*i])"""

    #rot_c_im, rot_d_im = get_vertical_image(sim, mjpy_model, camera_pos=[0, 0.4, 1.5])

    """force = [0, 0, -9.8]
                torque = [0, 0, 0]
                sim.data.xfrc_applied[sim.model._body_name2id["torso"], :] = force + torque"""


    #[result, depths, d_im], rot_c_im, rot_d_im = env.env.gqcnn(vis_on=False, num_candidates=10)

    from robosuite.gqcnn.policy_mujoco import GQCNN
    model_ver = 2
    model_name = None
    policy = GQCNN(model_ver, model_name)

    result, depths, d_im = policy.evaluate_gqcnn(rot_c_im, rot_d_im, vis_on=True, num_candidates=20)

    body_id = sim.model.body_name2id("base_link")
    base_pos = sim.data.body_xpos[body_id]
    base_quat = sim.data.body_xquat[body_id]

    target_id = sim.model.body_name2id("torso")
    target_pos = sim.data.body_xpos[target_id]
    target_quat = sim.data.body_xquat[target_id]

    pose_goal = geometry_msgs.msg.Pose()
    quat0 = [0.0, 0.7071068, 0.0, 0.7071068]
    quat1 = [0.7071068, 0.0, 0.0, 0.7071068]
    quat_mul = quaternion_multiply(quat1, quat0)

    pose_goal.orientation.x = -0.5
    pose_goal.orientation.y = 0.5
    pose_goal.orientation.z = 0.5
    pose_goal.orientation.w = 0.5
    pose_goal.position.x = target_pos[0]
    pose_goal.position.y = target_pos[1]
    pose_goal.position.z = target_pos[2] + 0.175
    move_to_target(sim, viewer, grasp=0.0, target_endpos=pose_goal)

    #get_vertical_image(sim)

    grasp_w_sensor(sim, viewer)

    get_vertical_image(sim, mjpy_model, camera_pos=[0, 0.4, 1.5])

    move_to_target(sim, viewer, grasp=1.0, target_qpos=initial_joint_qpos)
    for i in range(1000000):
        rospy.sleep(0.001)
        viewer.render()

if __name__ == '__main__':
    main()