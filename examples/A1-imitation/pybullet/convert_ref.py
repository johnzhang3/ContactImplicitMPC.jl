from cProfile import label
import numpy as np
import pybullet
import pybullet_data as pd
import retarget_config_a1 as config
from pybullet_utils import transformations
import os
import json
import matplotlib.pyplot as plt

"""
This script converts full body reference trajecory to centroidal 
reference trajectory.

full body : [pos, rot (quaternion), joint angles]
centroidal: [pos, rot (euler), foot positions]
"""

COM_OFFSET = -np.array([0.012731, 0.002186, 0.000515])
HIP_OFFSETS = np.array([[0.183, -0.047, 0.], [0.183, 0.047, 0.],
                        [-0.183, -0.047, 0.], [-0.183, 0.047, 0.]
                        ]) + COM_OFFSET
POS_SIZE = 3
ROT_SIZE = 4

centroidal_path = os.path.join(os.getcwd(), 'examples', 'A1-imitation', 'centroidal_ref_traj')
fullbody_path = os.path.join(os.getcwd(), 'examples', 'A1-imitation', 'fullbody_ref_traj')

def foot_position_in_hip_frame(angles, l_hip_sign=1):
  theta_ab, theta_hip, theta_knee = angles[0], angles[1], angles[2]
  l_up = 0.2
  l_low = 0.2
  l_hip = 0.08505 * l_hip_sign
  leg_distance = np.sqrt(l_up**2 + l_low**2 +
                         2 * l_up * l_low * np.cos(theta_knee))
  eff_swing = theta_hip + theta_knee / 2

  off_x_hip = -leg_distance * np.sin(eff_swing)
  off_z_hip = -leg_distance * np.cos(eff_swing)
  off_y_hip = l_hip

  off_x = off_x_hip
  off_y = np.cos(theta_ab) * off_y_hip - np.sin(theta_ab) * off_z_hip
  off_z = np.sin(theta_ab) * off_y_hip + np.cos(theta_ab) * off_z_hip
  return np.array([off_x, off_y, off_z])

def foot_positions_in_base_frame(foot_angles):
  foot_angles = foot_angles.reshape((4, 3))
  foot_positions = np.zeros((4, 3))
  for i in range(4):
    foot_positions[i] = foot_position_in_hip_frame(foot_angles[i],
                                                   l_hip_sign=(-1)**(i + 1))
  return foot_positions + HIP_OFFSETS

def save_traj(frames, out_filename, FRAME_DURATION):
  with open(out_filename, "w") as f:
    f.write("{\n")
    f.write("\"LoopMode\": \"Wrap\",\n")
    f.write("\"FrameDuration\": " + str(FRAME_DURATION) + ",\n")
    f.write("\"EnableCycleOffsetPosition\": true,\n")
    f.write("\"EnableCycleOffsetRotation\": true,\n")
    f.write("\n")

    f.write("\"Frames\":\n")

    f.write("[")
    for i in range(frames.shape[0]):
      curr_frame = frames[i]

      if i != 0:
        f.write(",")
      f.write("\n  [")

      for j in range(frames.shape[1]):
        curr_val = curr_frame[j]
        if j != 0:
          f.write(", ")
        f.write("%.5f" % curr_val)

      f.write("]")

    f.write("\n]")
    f.write("\n}")

  return

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def plot_centroidal_traj():
    '''plot foot height'''
    ref_list = ['pace', 'pace_backward', 'sidesteps','hopturn']
    # ref_list = ['pace']
    for ref in ref_list:
        cent_traj_path = os.path.join(centroidal_path, f"{ref}.json")
        with open(cent_traj_path, 'r') as f:
            cent_traj_dict = json.load(f)
        cent_traj = cent_traj_dict["Frames"]
        cent_traj = np.array(cent_traj)
        fig, axs = plt.subplots()
        # foot height
        axs.plot(cent_traj[:, 8], label = "foot 1")
        axs.plot(cent_traj[:, 11], label = "foot 2")
        axs.plot(cent_traj[:, 14], label = "foot 3")
        axs.plot(cent_traj[:, 17], label = "foot 4")
        axs.set_title(f'{ref} foot height')
        axs.legend()
        
        plt.savefig(os.path.join(centroidal_path, 'plots', f'{ref}_foot_height.png'))
        plt.cla()

def main():
    p = pybullet
    p.connect(p.GUI, options="--width=1920 --height=1080 --mp4=\"test.mp4\" --mp4fps=60")
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  
    pybullet.setAdditionalSearchPath(pd.getDataPath())
    robot = pybullet.loadURDF(config.URDF_FILENAME, config.INIT_POS, config.INIT_ROT)
    # robot = 
    ref_list = ['pace', 'pace_backward', 'sidesteps','hopturn']
    # ref_list = ['hopturn']
	
    for ref in ref_list:
        # load data
        fullbody_traj_path = os.path.join(fullbody_path, f"{ref}.txt")
        with open(fullbody_traj_path, 'r') as f:
            ref_motion_dict = json.load(f)
        ref_motion_frames = ref_motion_dict['Frames']
        ref_motion_frames = np.array(ref_motion_frames)
        num_frames = ref_motion_frames.shape[0]
        # ref_motion_traj = np.zeros((num_frames, nx))
        cent_traj = np.zeros((num_frames, 3+3+12))

        for i in range(num_frames):
            # body position
            cent_traj[i][0:POS_SIZE] = ref_motion_frames[i][0:POS_SIZE]
            # euler angle
            cent_traj[i][POS_SIZE:POS_SIZE*2] = transformations.euler_from_quaternion(ref_motion_frames[i][POS_SIZE:POS_SIZE+ROT_SIZE], axes='rzyx')
            # foot positions
            angles = ref_motion_frames[i][POS_SIZE+ROT_SIZE:]
            foot_pos_base = foot_positions_in_base_frame(angles)
            # TODO: add rotation matrix
            translation_matrix = transformations.translation_matrix(ref_motion_frames[i][0:POS_SIZE])

            rotation_matrix = transformations.quaternion_matrix(ref_motion_frames[i][POS_SIZE:POS_SIZE+ROT_SIZE])
            # rotation_matrix = quaternion_rotation_matrix(ref_motion_frames[i][POS_SIZE:POS_SIZE+ROT_SIZE])
            # angle, direction, point = transformations.rotation_from_matrix(rotation_matrix)

            foot_pos_world = np.dot(rotation_matrix[:3, :3] , foot_pos_base.T).T + ref_motion_frames[i][0:POS_SIZE] # this version kinda works
            
            # translation then rotation
            # foot_pos_world = np.dot(rotation_matrix[:3, :3] , (foot_pos_base + ref_motion_frames[i][0:POS_SIZE]).T).T

            
            # foot_pos_world = np.dot(foot_pos_base, rotation_matrix[:3, :3] ) + ref_motion_frames[i][0:POS_SIZE]
            # foot_pos_world = np.dot(foot_pos_base,rotation_matrix ) + ref_motion_frames[i][0:POS_SIZE]
            # foot_pos_world = foot_pos_base + ref_motion_frames[i][0:POS_SIZE]
            cent_traj[i][POS_SIZE*2:] = foot_pos_world.reshape(-1)
        
        centroidal_save_path = os.path.join(centroidal_path, f'{ref}.json')
        save_traj(cent_traj, centroidal_save_path, FRAME_DURATION=ref_motion_dict["FrameDuration"])
    

if __name__ == "__main__":
    main()
    plot_centroidal_traj()