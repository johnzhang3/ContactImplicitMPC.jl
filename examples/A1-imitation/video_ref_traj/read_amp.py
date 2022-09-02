import numpy as np
import json
import sys
import os 
from pybullet_utils import transformations
import matplotlib.pyplot as plt

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

def convert_video_traj(path, outdir, name):
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
    save_path = os.path.join(outdir, f"{name}.json")
    with open(save_path, 'w') as f:
        json.dump(out_dict, f, indent=4)

def plot_traj(outdir, num):

    cent_traj_path = os.path.join(outdir, f"{num}.json")
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
    axs.set_title('video foot height')
    axs.legend()
    plt.show()

if __name__ == "__main__":
    nums = [1005, 1013]
    # nums = [1013]
    for num in nums:
        path = os.path.join(os.getcwd(), "examples/A1-imitation/video_ref_traj", "trajs_txt", f"amp-shiba-haru-{num}.txt")
        outdir = os.path.join(os.getcwd(), "examples/A1-imitation/video_ref_traj", "trajs_json")

        convert_video_traj(path, outdir, num)
        # plot_traj(outdir, num)

