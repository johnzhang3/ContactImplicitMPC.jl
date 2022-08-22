import numpy as np
import json
import sys
import os 
from pybullet_utils import transformations

def parse_amp(amp_info):
    msm = {}
    msm['pos'] = amp_info[...,0:3] # root position
    msm['orn'] = amp_info[...,3:7] # root orientation (xyzw)
    msm['vel'] = amp_info[...,31:34] # root velocity
    msm['avel'] = amp_info[...,34:37] # root angular velocity
    msm['jang'] = amp_info[...,7:19] # joint angles (laikago)
    msm['jvel'] = amp_info[...,37:49] # joint angle velocity
    msm['kp'] = amp_info[...,61:73] # keypoint (4x3)
    msm['kp_vel'] = amp_info[...,73:85] # keypoint velocity
    return msm

def get_video_traj(path, outdir):
    Frames = []
    out_dict = {}
    with open(path, "r") as f:
        amp_info = json.load(f)
        samp_int = amp_info['FrameDuration']
        out_dict['FrameDuration'] = amp_info['FrameDuration']
        amp_info = np.asarray(amp_info['Frames'])
        for i in range(len(amp_info)):
            msm = parse_amp(amp_info[i])
            orn_euler = transformations.euler_from_quaternion(msm['orn'], axes='rzyx')

            frame = np.concatenate((msm['pos'], orn_euler, msm['kp']), 0)
            
            Frames.append(list(frame))
        out_dict['Frames'] = Frames
    save_path = os.path.join(outdir, "video_traj.json")
    with open(save_path, 'w') as f:
        json.dump(out_dict, f, indent=4)

    # np.savetxt('%s/out-root_traj.txt'%outdir, root_traj)
    # np.savetxt('%s/out-feet_traj.txt'%outdir, feet_traj)

if __name__ == "__main__":

    path = os.path.join(os.getcwd(), "examples/A1-imitation/from_gengshan", "amp-shiba-haru-1013.txt")
    outdir = os.path.join(os.getcwd(), "examples/A1-imitation/from_gengshan")

    get_video_traj(path, outdir)


