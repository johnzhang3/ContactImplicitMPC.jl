"""visualizes motion data on A1"""

import numpy as np
import pybullet
import pybullet_data as pd
# from motion_imitation.robots.a1 import URDF_FILENAME
# from retarget_motion.retarget_motion import *
import retarget_config_a1 as config
from pybullet_utils import transformations

# from retarget_motion.retarget_motion import set_pose
import os
import json
import time

# from retarget_motion.retarget_motion import FRAME_DURATION

POS_SIZE = 3
ROT_SIZE = 4
GROUND_URDF_FILENAME = "plane_implicit.urdf"

def set_root_pos(root_pos, pose):
  pose[0:POS_SIZE] = root_pos
  return

def set_root_rot(root_rot, pose):
  pose[POS_SIZE:(POS_SIZE + ROT_SIZE)] = root_rot
  return
def get_root_pos(pose):
  return pose[0:POS_SIZE]

def get_root_rot(pose):
  return pose[POS_SIZE:(POS_SIZE + ROT_SIZE)]

def set_pose(robot, pose):
  num_joints = pybullet.getNumJoints(robot)
  root_pos = get_root_pos(pose)
  root_rot = get_root_rot(pose)
  pybullet.resetBasePositionAndOrientation(robot, root_pos, root_rot)

  for j in range(num_joints):
    j_info = pybullet.getJointInfo(robot, j)
    j_state = pybullet.getJointStateMultiDof(robot, j)

    j_pose_idx = j_info[3]
    j_pose_size = len(j_state[0])
    j_vel_size = len(j_state[1])

    if (j_pose_size > 0):
      j_pose = pose[j_pose_idx:(j_pose_idx + j_pose_size)]
      j_vel = np.zeros(j_vel_size)
      pybullet.resetJointStateMultiDof(robot, j, j_pose, j_vel)

  return

def update_camera(robot):
  base_pos = np.array(pybullet.getBasePositionAndOrientation(robot)[0])
  [yaw, pitch, dist] = pybullet.getDebugVisualizerCamera()[8:11]
  pybullet.resetDebugVisualizerCamera(dist, yaw, pitch, base_pos)
  return

def main():

    p = pybullet
    p.connect(p.GUI, options="--width=1920 --height=1080 --mp4=\"test.mp4\" --mp4fps=60")
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)

    pybullet.setAdditionalSearchPath(pd.getDataPath())

    while True:
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.8)
        ground = pybullet.loadURDF(GROUND_URDF_FILENAME)
        robot = pybullet.loadURDF(config.URDF_FILENAME, config.INIT_POS, config.INIT_ROT)
        # from motion_imitation.robots import a1
        # env = env_builder.build_laikago_env( motor_control_mode = robot_config.MotorControlMode.HYBRID, enable_rendering=True)
        # env.reset()
        # robot = a1(pybullet, URDF_FILENAME)
        # Set robot to default pose to bias knees in the right direction.
        set_pose(robot, np.concatenate([config.INIT_POS, config.INIT_ROT, config.DEFAULT_JOINT_POSE]))
        p.removeAllUserDebugItems()
        # ref_list = ['dog_pace', 'dog_trot', 'dog_spin', 'dog_backwards_pace', 'dog_backwards_trot', 
        #             'inplace_steps', 'hopturn', 'runningman', 'sidesteps', 'turn']
        # ref_list = ['pace', 'pace_backward', 'sidesteps','hopturn']
        ref_list = ['hopturn']
        for ref in ref_list:
            # load data
            # ref_motion_path = os.path.join(os.getcwd(), 'motion_imitation', 'data', 'motions', f"{ref}.txt")
            ref_motion_path = os.path.join(os.getcwd(), 'motion_imitation', 'data', 'motions_new', f"{ref}.txt")
            with open(ref_motion_path, 'r') as f:
                ref_motion_dict = json.load(f)
            ref_motion_frames = ref_motion_dict['Frames']
            ref_motion_frames = np.array(ref_motion_frames)
            # FRAME_DURATION = ref_motion_dict['FrameDuration']
            FRAME_DURATION = 0.1
            repeat = 1
            num_frames = ref_motion_frames.shape[0]
            # init_quaternion = np.array([0.5,0.5,0.5,0.5])
            # init_euler = transformations.euler_from_quaternion(init_quaternion, axes='rzyx')

            f = 0
            for i in range(repeat * num_frames):
                time_start = time.time()
                f_idx = f%num_frames
                pose = ref_motion_frames[f_idx]

                # current_euler = transformations.euler_from_quaternion(pose[POS_SIZE:(POS_SIZE + ROT_SIZE)], axes='rzyx')
                # current_euler = np.array(current_euler)
                # relative_euler = current_euler - init_euler
                # pose[POS_SIZE:(POS_SIZE + ROT_SIZE)] = transformations.quaternion_from_euler(relative_euler[0], relative_euler[1], relative_euler[2], axes='rzyx')
                set_pose(robot, pose)
                # update_camera(robot)
                p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
                f += 1
                time_end = time.time()
                sleep_dur = FRAME_DURATION - (time_end - time_start)
                sleep_dur = max(0, sleep_dur)
            
                time.sleep(sleep_dur)
        # print('done')




if __name__ == "__main__":
    # tf.app.run(main)
    main()