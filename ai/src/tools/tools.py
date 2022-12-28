import os
import numpy as np
import matplotlib.pyplot as plt

def backward_relative_path(target_dir):
    rel_path = target_dir
    while not os.path.isdir(rel_path):
        rel_path = os.path.join('..', rel_path)
    print(f"Backward relative path of <{target_dir}> found: <{rel_path}>")
    return rel_path

def plot(M, title="", y1_max=None, y2_max=None, y3_max=None, y4_max=None,):
    plt.figure(title, figsize=(13,6))
    plt.subplots_adjust(wspace=.3, hspace=0.2)

    ax1 = plt.subplot(121,label='ax1')
    plt1 = ax1.plot(M[:,0],M[:,1],'.-b', label='x [mm]')
    y1max = np.abs(M[:,1]).max()*1.1 if y1_max is None else y1_max
    ax1.set_ylim(-y1max, y1max)
    ax1.set_xlabel('time [ms]')
    ax1.tick_params(axis='y', labelcolor='b')
    ax1.set_title('Linear displacement')
    ax1.grid()
    ax2 = ax1.twinx()
    plt2 = ax2.plot(M[:,0],M[:,2],'.-m', label="x' [m/s]")
    y2max = np.abs(M[:,2]).max()*1.1 if y2_max is None else y2_max
    ax2.set_ylim(-y2max, y2max)
    ax2.tick_params(axis='y', labelcolor='m')
    all_plots = plt1+plt2
    labs = [p.get_label() for p in all_plots]
    ax1.legend(all_plots, labs, loc='upper right', fontsize=10)

    ax3 = plt.subplot(122,label='ax1')
    plt3 = ax3.plot(M[:,0],M[:,3],'.-g',label=r'$\theta$'+' [°]')
    ax3.set_ylim(0,360)
    ax3.set_xlabel('time [ms]')
    ax3.tick_params(axis='y', labelcolor='g')
    ax3.set_title('Angular displacement')
    ax3.grid()
    ax4 = ax3.twinx()
    plt4 = ax4.plot(M[:,0],M[:,4],'.-r',label=r"$\theta'$"+ ' [°/s]')
    y4max = np.abs(M[:,4]).max()*1.1 if y4_max is None else y4_max
    ax4.set_ylim(-y4max, y4max)
    ax4.tick_params(axis='y', labelcolor='r')
    all_plots = plt3+plt4
    labs = [p.get_label() for p in all_plots]
    ax3.legend(all_plots, labs, loc='upper right', fontsize=10);


if __name__ == "__main__":

    backward_relative_path("tools")
