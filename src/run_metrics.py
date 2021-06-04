from datetime import time
import numpy as np
import math
import scipy.io as sio
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

import rospy
import rosbag
import os
from datetime import datetime

mpl.style.use('seaborn')
plt.rcParams["lines.markeredgewidth"] = 0.5

class PoseData:
    def __init__(self, positions, orientations, times, covs=None):
        self.positions = positions
        self.orientations = orientations
        self.times = times
        self.covariances = covs

class GNSSData:
    def __init__(self, positions, time):
        self.pos = positions
        self.time = time

def mat2GNSSData(filename):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    gnss_pos = m['simple_tunnel_ds']['gnss']['measurements']
    gnss_time = m['simple_tunnel_ds']['gnss']['times']
    return GNSSData(gnss_pos, gnss_time)

def plot_cov_ellipse2d(
    ax: plt.Axes,
    mean: np.ndarray = np.zeros(2),
    cov: np.ndarray = np.eye(2),
    yaw: float = 0,
    n_sigma: float = 1,
    *,
    edgecolor: "Color" = "C0",
    facecolor: "Color" = "none",
    **kwargs,  # extra Ellipse keyword arguments
) -> mpl.patches.Ellipse:
    """Plot a n_sigma covariance ellipse centered in mean into ax."""
    try:
        ell_trans_mat = np.zeros((3, 3))
        ell_trans_mat[:2, :2] = np.linalg.cholesky(cov)
        ell_trans_mat[:2, 2] = mean
        ell_trans_mat[2, 2] = 1

        ell = mpl.patches.Ellipse(
            (0.0, 0.0),
            2.0 * n_sigma,
            2.0 * n_sigma,
            edgecolor=edgecolor,
            facecolor=facecolor,
            angle=np.rad2deg(yaw),
            **kwargs,
        )
        trans = mpl.transforms.Affine2D(ell_trans_mat)
        ell.set_transform(trans + ax.transData)
        return ax.add_patch(ell)
    except:
        print("i")

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

def bag2numpy(bagname):
    estimate_positions = []
    estimate_orientations = []
    estimate_times = []
    estimate_covariances = []

    gt_positions = []
    gt_orientations = []
    gt_times = []
    
    with rosbag.Bag(bagname, 'r') as bag:
        for msg in bag.read_messages(['/pose', '/ground_truth']):
            if msg.topic == '/ground_truth':
                position = msg.message.pose.position
                positionArr = [position.x, position.y, position.z] 
                quat = msg.message.pose.orientation
                angles = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
                gt_positions.append(positionArr)
                gt_orientations.append([angles[0], angles[1], angles[2]]) 
                gt_times.append(rospy.Time.to_sec(msg.message.header.stamp))
            elif msg.topic == '/pose':
                position = msg.message.pose.pose.position
                positionArr = [position.x, position.y, position.z]
                quat = msg.message.pose.pose.orientation
                angles = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
                estimate_positions.append(positionArr)
                estimate_orientations.append(angles)
                estimate_times.append(rospy.Time.to_sec(msg.message.header.stamp))
                estimate_covariances.append(msg.message.pose.covariance)
    est = PoseData(np.array(estimate_positions), np.array(estimate_orientations), np.array(estimate_times), np.array(estimate_covariances).reshape(-1, 6, 6))
    gt = PoseData(np.array(gt_positions), np.array(gt_orientations), np.array(gt_times))
    return est, gt

def serialize(data, col:int):
    serialized = np.array(np.float_(data[0, col].split(';')))
    for row in data[1:, col]:
        serialized = np.vstack((serialized, np.float_(row.split(';'))))
    return serialized
        

def csv2numpy(filename: str, other_estimate: PoseData):
    data = pd.read_csv(filename, delimiter=",")
    
    unserializedPoses = data[data["key"].str.contains(pat="x")].to_numpy()
    unserializedLandmarks = data[data["key"].str.contains(pat="l")].to_numpy()
    unserializedVelocitites = data[data["key"].str.contains(pat="v")].to_numpy()
    unserializedBiases = data[data["key"].str.contains(pat="b")].to_numpy()
    serPoses = serialize(unserializedPoses, 2)
    serLandmarks = serialize(unserializedLandmarks, 1)
    if unserializedVelocitites.shape[0] > 0:
        serVelocities = serialize(unserializedVelocitites, 3)
        serBiases = serialize(unserializedBiases, 4)
    else:
        serVelocities = []
        serBiases = []

    estimate_positions=serPoses[:, :3]
    estimate_orientations=serPoses[:, 3:6]
    estimate_covariances=serPoses[:, 6:]
    estimate_times = np.concatenate(([0], other_estimate.times))
    estimates = PoseData(estimate_positions, estimate_orientations, estimate_times, estimate_covariances.reshape(-1, 6, 6))
    return estimates, serVelocities, serLandmarks, serBiases

def plotTrajectory2D(estimatesBeforeSmoothing:PoseData, estimatesAfterSmoothing:PoseData, gts:PoseData, skipPlt=4, title="", ylim=[-10, 10], xlim=[0, 240], lastGT=np.iinfo(np.int64).max):
    fig, ax = plt.subplots(3, 1, clear=True, figsize=(10,10), sharex=True)
    fig.suptitle(title)
    ax[0].set_title("Position")
    ax[0].plot(gts.positions[:lastGT+1, 0], gts.positions[:lastGT+1, 1], color='C0', label=r'$Ground Truth$')
    ax[0].plot(estimatesBeforeSmoothing.positions[:, 0], estimatesBeforeSmoothing.positions[:, 1], color='C1', label=r'$Before Smoothing$')
    ax[0].plot(estimatesAfterSmoothing.positions[:, 0], estimatesAfterSmoothing.positions[:, 1], color='C2', label=r'$After Smoothing$')
    ax[0].legend()
    ax[0].set_ylabel("West [m]")
    #ax[0].set_xlabel("North [m]")
    ax[0].set_xlim(*xlim)
    ax[0].set_ylim(*ylim)

    #fig2, ax2 = plt.subplots(clear=True, figsize=(10,10))
    ax[1].set_title("Before Smoothing with Covariance")
    for i, (position, orientation, cov) in enumerate(zip(estimatesBeforeSmoothing.positions, estimatesBeforeSmoothing.orientations, estimatesBeforeSmoothing.covariances)):
        if i%skipPlt == 0:
            #plot_cov = np.array(([[cov[1, 1], cov[1, 0]], [cov[0, 1], cov[0, 0]]]))
            plot_cov = cov[:2, :2]
            plot_cov_ellipse2d(ax[1], np.array([position[0], position[1]]), plot_cov, yaw = orientation[-1], edgecolor='r')
    ax[1].plot(estimatesBeforeSmoothing.positions[:, 0], estimatesBeforeSmoothing.positions[:, 1], color="C1", marker="x", label='XYPos', markevery=1)
    ax[1].legend()
    ax[1].set_ylabel("West [m]")
    #ax[1].set_xlabel("North [m]")
    ax[1].set_xlim(*xlim)
    ax[1].set_ylim(*ylim)

    ax[2].set_title("After Smoothing with Covariance")
    for i, (position, orientation, cov) in enumerate(zip(estimatesAfterSmoothing.positions, estimatesAfterSmoothing.orientations, estimatesAfterSmoothing.covariances)):
        if i%skipPlt == 0:
            #plot_cov = np.array(([[cov[1, 1], cov[1, 0]], [cov[0, 1], cov[0, 0]]]))
            plot_cov = cov[:2, :2]
            plot_cov_ellipse2d(ax[2], np.array([position[0], position[1]]), plot_cov, yaw = orientation[-1], edgecolor='r')
    ax[2].plot(estimatesAfterSmoothing.positions[:, 0], estimatesAfterSmoothing.positions[:, 1], color='C2' ,marker="x", label='XYPos', markevery=1)
    ax[2].legend()
    ax[2].set_ylabel("West [m]")
    ax[2].set_xlabel("North [m]")
    ax[2].set_xlim(*xlim)
    ax[2].set_ylim(*ylim)
    return fig

def plotExtraStateEstimates(time, velocities, biases):
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10,10))
    axs[0].set_title("Speed")
    axs[0].plot(time, np.linalg.norm(velocities, ord=2, axis=1))
    axs[0].legend([r"$V$"])
    axs[1].set_title("Accelerometer biases")
    axs[1].plot(time, biases[:, :3])
    axs[1].legend([r"$a_{xb}$", r"$a_{yb}$", r"$a_{zb}$"])
    axs[2].set_title("Gyro biases")
    axs[2].plot(time, biases[:, 3:])
    axs[2].legend([r"$p_b$", r"$q_b$", r"$r_b$"])
    return fig

def plotMapWithTrajectory(positions, landmarks, view_init=[5, 170], xlim=[-1, 250], ylim=[-20, 20], zlim=[-0.2, 30]):
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(projection="3d")
    ax.set_title("Estimated Map with Trajectory")
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="C2")
    ax.scatter(landmarks[:, 0], landmarks[:, 1], landmarks[:, 2])
    ax.set_xlim3d(*xlim)
    ax.set_ylim3d(*ylim)
    ax.set_zlim3d(*zlim)
    ax.set_xlabel("North [m]")
    ax.set_ylabel("West [m]")
    ax.set_zlabel("Up [m]")
    ax.view_init(*view_init)
    # Create cubic bounding box to simulate equal aspect ratio
    
    """xmax = positions[:, 0].max()
    xmin = positions[:, 0].min()
    ymax = positions[:, 1].max()
    ymin = positions[:, 1].min()
    zmax = positions[:, 2].max()
    zmin = positions[:, 2].min()

    max_range = np.array([xmax-xmin, ymax-ymin, zmax-zmin]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(xmax+xmin)
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(ymax+ymin)
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(zmax+ zmin)
    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')"""
    return fig

def plotErrorsOverTime(estimatesBeforeSmoothing, estimatesAfterSmoothing, gts, matchesBeforeSmoothing, matchesAfterSmoothing, greyArea=[]):

    def angleError(alpha, beta):
        phi = np.abs(beta-alpha) % 360
        distance = 360-phi if phi > 180 else phi
        return distance

    func = np.vectorize(angleError)
    fig, axs = plt.subplots(2, 1, clear=True, figsize=(10,10), sharex=True)
    fig.suptitle(" Absolute Errors", fontsize=25)

    # Before smoothing
    axs[0].plot(gts.times[matchesBeforeSmoothing], np.linalg.norm(estimatesBeforeSmoothing.positions - gts.positions[matchesBeforeSmoothing], ord=2, axis=1), color="C1", label="Before Smoothing")
    if len(greyArea) > 0:
        axs[0].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        axs[0].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axs[0].set_xlim([0, gts.times[-1]])

    axs[0].set_title("XY Position Error")
    axs[0].set_ylabel("Error [m]")
    axs[1].plot(gts.times[matchesBeforeSmoothing], func(np.rad2deg(estimatesBeforeSmoothing.orientations[:, -1]), np.rad2deg(gts.orientations[matchesBeforeSmoothing, -1])), color="C1", label="Before Smoothing")
    if len(greyArea) > 0:
        axs[1].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        axs[1].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axs[1].set_xlim([0, gts.times[-1]])
    axs[1].set_ylabel("Error [deg]")
    axs[1].set_title("Heading Error")

    # After smoothing
    axs[0].plot(gts.times[matchesAfterSmoothing], np.linalg.norm(estimatesAfterSmoothing.positions - gts.positions[matchesAfterSmoothing], ord=2, axis=1), color="C2", label="After Smoothing")
    axs[1].plot(gts.times[matchesAfterSmoothing], func(np.rad2deg(estimatesAfterSmoothing.orientations[:, -1]), np.rad2deg(gts.orientations[matchesAfterSmoothing, -1])), color="C2", label="After Smoothing")
    axs[0].legend()
    axs[1].legend()

    plt.xlabel("Time [s]")

    return fig

def plotNEES(gts:PoseData, estimatesBeforeSmoothing:PoseData, estimatesAfterSmoothing:PoseData, matchesBeforeSmoothing, matchesAfterSmoothing, greyArea=[]):
    import scipy.stats

    def angleError(alpha, beta):
        phi = np.abs(beta-alpha) % (2*np.pi)
        distance = 2*np.pi-phi if phi > np.pi else phi
        return distance

    confprob = 0.95
    CI3 = np.array(scipy.stats.chi2.interval(confprob, 3)).reshape((2,1))
    CI2 = np.array(scipy.stats.chi2.interval(confprob, 2)).reshape((2,1))
    CI1 = np.array(scipy.stats.chi2.interval(confprob, 1)).reshape((2,1))
    fig, ax = plt.subplots(3, 1, clear=True, figsize=(10,10), sharex=True)
    #fig.suptitle("Planar NEES Over Time")
    NEESTotalBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    NEESPosBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    NEESYawBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    for i, (est_pos, est_ori, est_cov) in enumerate(zip(estimatesBeforeSmoothing.positions, estimatesBeforeSmoothing.orientations, estimatesBeforeSmoothing.covariances)):
        gt_pos = gts.positions[matchesBeforeSmoothing[i]]
        gt_ori = gts.orientations[matchesBeforeSmoothing[i]]

        # XY
        e_pos = est_pos[:2] - gt_pos[:2]
        P = np.array(([[est_cov[1, 1], est_cov[1, 0]], [est_cov[0, 1], est_cov[0, 0]]]))
        P_inv = np.linalg.inv(P)
        NEESPosBeforeSmoothing[i] = e_pos.T @ P_inv @ e_pos

        # Yaw
        e_yaw = angleError(est_ori[-1], gt_ori[-1])
        cov_yaw = est_cov[-1,-1]
        NEESYawBeforeSmoothing[i] = e_yaw*e_yaw / cov_yaw

        #Total
        P = np.array(([[est_cov[1,1], est_cov[1, 0], est_cov[1, -1]], [est_cov[0, 1], est_cov[0, 0], est_cov[0, -1]], [est_cov[-1, 1], est_cov[-1, 0], est_cov[-1, -1]]]))
        P_inv = np.linalg.inv(P)
        e_total = np.array([*e_pos, e_yaw])
        NEESTotalBeforeSmoothing[i] = e_total.T @ P_inv @ e_total

    insideCIPosBefore = np.mean((CI2[0] <= NEESPosBeforeSmoothing) * (NEESPosBeforeSmoothing <= CI2[1]))
    insideCIYawBefore = np.mean((CI1[0] <= NEESYawBeforeSmoothing) * (NEESYawBeforeSmoothing <= CI1[1]))
    insideCITotalBefore = np.mean((CI3[0] <= NEESTotalBeforeSmoothing) * (NEESTotalBeforeSmoothing <= CI3[1]))
    ax[0].plot(estimatesBeforeSmoothing.times, NEESPosBeforeSmoothing, color="C1", label="Before Smoothing")
    ax[1].plot(estimatesBeforeSmoothing.times, NEESYawBeforeSmoothing, color="C1", label="Before Smoothing")
    ax[2].plot(estimatesBeforeSmoothing.times, NEESTotalBeforeSmoothing, color="C1", label="Before Smoothing")

    NEESTotalAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    NEESPosAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    NEESYawAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    for i, (est_pos, est_ori, est_cov) in enumerate(zip(estimatesAfterSmoothing.positions, estimatesAfterSmoothing.orientations, estimatesAfterSmoothing.covariances)):
        gt_pos = gts.positions[matchesAfterSmoothing[i]]
        gt_ori = gts.orientations[matchesAfterSmoothing[i]]

        # XY
        e_pos = est_pos[:2] - gt_pos[:2]
        P = np.array(([[est_cov[1, 1], est_cov[1, 0]], [est_cov[0, 1], est_cov[0, 0]]]))
        P_inv = np.linalg.inv(P)
        NEESPosAfterSmoothing[i] = e_pos.T @ P_inv @ e_pos

        # Yaw
        e_yaw = angleError(est_ori[-1], gt_ori[-1])
        cov_yaw = est_cov[-1,-1]
        NEESYawAfterSmoothing[i] = e_yaw*e_yaw / cov_yaw
        #print(e_yaw)

        # Total
        P = np.array(([[est_cov[1,1], est_cov[1, 0], est_cov[1, -1]], [est_cov[0, 1], est_cov[0, 0], est_cov[0, -1]], [est_cov[-1, 1], est_cov[-1, 0], est_cov[-1, -1]]]))
        P_inv = np.linalg.inv(P)
        e_total = np.array([*e_pos, e_yaw])
        NEESTotalAfterSmoothing[i] = e_total.T @ P_inv @ e_total


    insideCIPosAfter = np.mean((CI2[0] <= NEESPosAfterSmoothing) * (NEESPosAfterSmoothing <= CI2[1]))
    insideCIYawAfter = np.mean((CI1[0] <= NEESYawAfterSmoothing) * (NEESYawAfterSmoothing <= CI1[1]))
    insideCITotalAfter = np.mean((CI3[0] <= NEESTotalAfterSmoothing) * (NEESTotalAfterSmoothing <= CI3[1]))


    ax[0].set_title(f"XY Position NEES\nBefore Smoothing: ({insideCIPosBefore*100:.1f} inside {100 * confprob} confidence interval)\nAfter Smoothing: ({insideCIPosAfter*100:.1f} inside {100 * confprob} confidence interval)")
    ax[1].set_title(f"Heading NEES\nBefore Smoothing: ({insideCIYawBefore*100:.1f} inside {100 * confprob} confidence interval)\nAfter Smoothing: ({insideCIYawAfter*100:.1f} inside {100 * confprob} confidence interval)")
    ax[2].set_title(f"Total Planar NEES\nBefore Smoothing: ({insideCITotalBefore*100:.1f} inside {100 * confprob} confidence interval)\nAfter Smoothing: ({insideCITotalAfter*100:.1f} inside {100 * confprob} confidence interval)")


    ax[0].plot(estimatesAfterSmoothing.times, NEESPosAfterSmoothing, color="C2", label="After Smoothing")
    ax[1].plot(estimatesAfterSmoothing.times, NEESYawAfterSmoothing, color="C2", label="After Smoothing")
    ax[2].plot(estimatesAfterSmoothing.times, NEESTotalAfterSmoothing, color="C2", label="After Smoothing")


    ax[0].plot(estimatesAfterSmoothing.times, np.repeat(CI2[0], estimatesAfterSmoothing.times.shape[0]), color="C3", label="Lower Bound")
    ax[0].plot(estimatesAfterSmoothing.times, np.repeat(CI2[1], estimatesAfterSmoothing.times.shape[0]), color="C4", label="Upper Bound")
    ax[1].plot(estimatesAfterSmoothing.times, np.repeat(CI1[0], estimatesAfterSmoothing.times.shape[0]), color="C3", label="Lower Bound")
    ax[1].plot(estimatesAfterSmoothing.times, np.repeat(CI1[1], estimatesAfterSmoothing.times.shape[0]), color="C4", label="Upper Bound")
    ax[2].plot(estimatesAfterSmoothing.times, np.repeat(CI3[0], estimatesAfterSmoothing.times.shape[0]), color="C3", label="Lower Bound")
    ax[2].plot(estimatesAfterSmoothing.times, np.repeat(CI3[1], estimatesAfterSmoothing.times.shape[0]), color="C4", label="Upper Bound")
    if len(greyArea) > 0:
        #ax.axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        #ax.axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
        ax[0].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        ax[0].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
        ax[1].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        ax[1].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
        ax[2].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        ax[2].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    #ax.set_xlim([0, gts.times[-1]])
    #ax.legend()
    ax[0].set_xlim([0, gts.times[-1]])
    ax[0].legend(loc='upper left')
    ax[1].set_xlim([0, gts.times[-1]])
    ax[1].legend(loc='upper left')
    ax[2].set_xlim([0, gts.times[-1]])
    ax[2].legend(loc='upper left')
    fig.tight_layout(pad=3.0)
    return fig


def plotExperiment(gts, imfile, matfile, imExtent=[-100, 450, -20, 30]):
    gnss = mat2GNSSData(matfile)
    im = plt.imread(imfile)
    fig, ax = plt.subplots(figsize=(10,3)) #Looks better with this size
    ax.imshow(im, extent=imExtent, aspect='auto')
    ax.plot(gts.positions[:, 0], gts.positions[:, 1])
    ax.scatter(gnss.pos[:, 0], gnss.pos[:, 1], marker="x", color="r")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("West [m]")
    #plt.savefig("experiment.pdf", format="pdf", bbox_inches="tight")
    return fig

def plotIMUvsRealLinAcc(filename:str):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    imu_linacc = m['simple_tunnel_ds']['imu']['accelerometer'].T
    imu_time = m['simple_tunnel_ds']['imu']['time']
    imu_gyro = m['simple_tunnel_ds']['imu']['gyroscope'].T
    linacc = m['simple_tunnel_ds']['linear_acc']['signals']['values']
    angularvel = m['simple_tunnel_ds']['angular_rate']['signals']['values']
    time = m['simple_tunnel_ds']['angular_rate']['time']

    fig, ax = plt.subplots(2, 1, figsize=(10,10)) #Looks better with this size
    ax[0].plot(imu_time, imu_linacc, label=r"$a_{imu}$")
    ax[0].plot(time, linacc, "--", label=r"$a_{true}$")
    ax[0].legend()
    ax[0].set_title("Accelerometer")
    ax[1].plot(imu_time, imu_gyro, label=r"$\omega_{imu}$")
    ax[1].plot(time, angularvel, "--", label=r"$\omega_{true}$")
    ax[1].legend()
    ax[1].set_title("Gyroscope")
    
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H:%M:%S")
    figfolder = f"../simulator_matlab/"

    fig.savefig(figfolder + "/imuVsTruth.pdf", format='pdf', bbox_inches="tight")

def main():
    #filepath = "../data/recorded_runs/python_plots/Saved/test/lidar/"
    #filepath = "../data/recorded_runs/python_plots/Saved/newestSensorComparisons/lidar+imu+gnss/"
    filepath = "../data/"
    matfile = "../data/april_2021/SimpleTunnel_BigLoop_ds.mat"
    imfile = "../simulator_matlab/straightTunnel_long.png"
    #estimates, gts = bag2numpy(f'simpleTunnel_big_loopClosure.bag')
    estimates, gts = bag2numpy(f'rawEstimates.bag')
    estimatesAfterSmoothing, velocities, landmarks, biases = csv2numpy(f"{filepath}LatestRun.csv", estimates)
    #run2TunnelStart = estimates.times[108]
    #run2TunnelEnd = estimates.times[175]
    #run1TunnelEnd = estimates.times[64]
    #run1TunnelStart = estimates.times[4]
    #greyArea = [run1TunnelStart, run1TunnelEnd, run2TunnelStart, run2TunnelEnd]
    greyArea=[]
    SmoothingEstVsGt = plotTrajectory2D(estimates, estimatesAfterSmoothing, gts, skipPlt=2, ylim=[-50, 50], xlim=[-10, 350], title="Estimated Position Vs True Position")
    #afterSmoothingEstVsGt = plotTrajectory2D(estimatesAfterSmoothing, gts, skipPlt=2, xlim=[-50, 50], ylim=[-10, 350], title="After Smoothing")
    est_to_gt_before_smoothing = np.zeros(estimates.times.shape, dtype=np.int)

    for i, est_t in enumerate(estimates.times):
        est_to_gt_before_smoothing[i] = np.argmax(gts.times > est_t)
    
    est_to_gt_after_smoothing = np.zeros(estimatesAfterSmoothing.times.shape, dtype=np.int)

    for i, est_t in enumerate(estimatesAfterSmoothing.times):
        est_to_gt_after_smoothing[i] = np.argmax(gts.times > est_t)

    errorFig = plotErrorsOverTime(estimates, estimatesAfterSmoothing, gts, est_to_gt_before_smoothing, est_to_gt_after_smoothing, greyArea=greyArea)
    NeesFig = plotNEES(gts, estimates, estimatesAfterSmoothing, est_to_gt_before_smoothing, est_to_gt_after_smoothing, greyArea=greyArea)
    estimatedMapWithTrajectory = plotMapWithTrajectory(estimatesAfterSmoothing.positions, landmarks)

    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H:%M:%S")
    figfolder = f"../data/recorded_runs/python_plots/RawRuns/{dt_string}"
    if not os.path.isdir(figfolder):
        os.makedirs(figfolder)
    
    saveConfig = {
        "format":"pdf", 
        "bbox_inches":"tight", 
        "dpi":1200
        }

    SmoothingEstVsGt.savefig(
        figfolder + "/SmoothingEstVsGt.pdf", 
        **saveConfig
        )
    errorFig.savefig(
        figfolder + "/absolutePositionError.pdf",
        **saveConfig
        )
    estimatedMapWithTrajectory.savefig(
        figfolder + "/estimatedMapWithTrajectory.pdf",
        **saveConfig
        )
    NeesFig.savefig(
        figfolder + "/planarNEES.pdf",
        **saveConfig
        )
    if len(velocities) > 0:
        estimatedExtraStateEstimates = plotExtraStateEstimates(estimatesAfterSmoothing.times, velocities, biases)
        estimatedExtraStateEstimates.savefig(
            figfolder + "/estimatedExtraStateEstimates.pdf",
            **saveConfig 
            )
    
    fig, ax = plt.subplots(2, 1, figsize=(10,10))
    ax[1].plot(gts.times[est_to_gt_after_smoothing], gts.positions[est_to_gt_after_smoothing, 1] - estimatesAfterSmoothing.positions[:, 1], label="Y Error After Smoothing")
    ax[1].plot(estimates.times, gts.positions[est_to_gt_before_smoothing, 1] - estimates.positions[:, 1], label="Y Error Before Smoothing")
    ax[0].plot(estimatesAfterSmoothing.times, gts.positions[est_to_gt_after_smoothing, 0] - estimatesAfterSmoothing.positions[:, 0], label="X Error After Smoothing")
    ax[0].plot(estimates.times, gts.positions[est_to_gt_before_smoothing, 0] - estimates.positions[:, 0], label="X Error Before Smoothing")
    #ax.plot(gts.times, gts.positions[:, 0], label="True")
    # # #ax.plot(estimatesAfterSmoothing.times, estimatesAfterSmoothing.positions[:, 0], label="Estimated")
    # ax[0].legend()
    # ax[1].legend()

    # fig, ax = plt.subplots(1, 1, figsize=(10,10))
    # ax.plot(gts.times, np.rad2deg(gts.orientations[:, 2]), label="True")
    # ax.plot(estimates.times, np.rad2deg(estimates.orientations[:, 2]), label="Estimate")
    # ax.legend()

    imExtent = [-100, 350, -20, 30]
    experimentFig = plotExperiment(gts, imfile, matfile, imExtent=imExtent)
    experimentFig.savefig(
        figfolder + "/experiment.pdf",
        **saveConfig
        )
    plt.show()

if __name__ == "__main__":
    main()
    #matfile = "../data/april_2021/SimpleTunnel_BigLoop_WithTrueAccAndAngularRate.mat"
    #plotIMUvsRealLinAcc(matfile)