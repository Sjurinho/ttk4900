import numpy as np
import math
import scipy.io as sio
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd

import rospy
import rosbag

from geometry_msgs.msg import Vector3, PoseStamped, Point, Quaternion, PoseWithCovarianceStamped

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
) -> matplotlib.patches.Ellipse:
    """Plot a n_sigma covariance ellipse centered in mean into ax."""
    try:
        ell_trans_mat = np.zeros((3, 3))
        ell_trans_mat[:2, :2] = np.linalg.cholesky(cov)
        ell_trans_mat[:2, 2] = mean
        ell_trans_mat[2, 2] = 1

        ell = matplotlib.patches.Ellipse(
            (0.0, 0.0),
            2.0 * n_sigma,
            2.0 * n_sigma,
            edgecolor=edgecolor,
            facecolor=facecolor,
            angle=np.rad2deg(yaw),
            **kwargs,
        )
        trans = matplotlib.transforms.Affine2D(ell_trans_mat)
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
    print(unserializedVelocitites.shape)
    unserializedBiases = data[data["key"].str.contains(pat="b")].to_numpy()
    serPoses = serialize(unserializedPoses, 2)
    if unserializedVelocitites.shape[0] > 0:
        serVelocities = serialize(unserializedVelocitites, 3)
        serLandmarks = serialize(unserializedLandmarks, 1)
        serBiases = serialize(unserializedBiases, 4)
    else:
        serVelocities = []
        serLandmarks = []
        serBiases = []

    estimate_positions=serPoses[:, :3]
    estimate_orientations=serPoses[:, 3:6]
    estimate_covariances=serPoses[:, 6:]
    estimate_times = np.linspace(other_estimate.times[0], other_estimate.times[-1], estimate_positions.shape[0])
    estimates = PoseData(estimate_positions, estimate_orientations, estimate_times, estimate_covariances.reshape(-1, 6, 6))
    return estimates, serVelocities, serLandmarks, serBiases

def plotTrajectory2D(estimates:PoseData, gts:PoseData, skipPlt=4, title="", xlim=[-10, 10], ylim=[0, 240]):
    fig, ax = plt.subplots(1, 2, clear=True, figsize=(10,10), sharey=True)
    fig.suptitle(title)
    ax[0].set_title("Estimate vs Ground Truth")
    ax[0].plot(estimates.positions[:, 1], estimates.positions[:, 0], label=r'$\hat{x}$')
    ax[0].plot(gts.positions[:, 1], gts.positions[:, 0], label=r'$x$')
    ax[0].legend()
    ax[0].set_xlim(*xlim)
    ax[0].set_ylim(*ylim)

    #fig2, ax2 = plt.subplots(clear=True, figsize=(10,10))
    ax[1].set_title("Estimate with Covariance")
    for i, (position, orientation, cov) in enumerate(zip(estimates.positions, estimates.orientations, estimates.covariances)):
        if i%skipPlt == 0:
            plot_cov = np.array(([[cov[1, 1], cov[1, 0]], [cov[0, 1], cov[0, 0]]]))
            plot_cov_ellipse2d(ax[1], np.array([position[1], position[0]]), plot_cov, yaw = orientation[-1], edgecolor='r')
    ax[1].plot(estimates.positions[:, 1], estimates.positions[:, 0], marker="x", label='XYPos', markevery=1)
    ax[1].legend()
    ax[1].set_xlim(*xlim)
    ax[1].set_ylim(*ylim)
    return fig#, fig2

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
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="orange")
    ax.scatter(landmarks[:, 0], landmarks[:, 1], landmarks[:, 2])
    ax.set_xlim3d(*xlim)
    ax.set_ylim3d(*ylim)
    ax.set_zlim3d(*zlim)
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

def plotErrorsOverTime(estimates, gts, matches):
    fig, ax = plt.subplots(1, 1, clear=True, figsize=(10,10), sharey=True)
    print(gts.times[matches].shape, np.linalg.norm(estimates.positions - gts.positions[matches]))
    ax.plot(gts.times[matches], np.linalg.norm(estimates.positions - gts.positions[matches], axis=1))
    ax.set_title("Absolute position error")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    #ax.set_xlabel("Time [s]")
    #ax.set_ylabel("error [m]")
    #plt.show()
    return fig

def plotExperiment(gts, imfile, matfile, imExtent=[-100, 450, -20, 20]):
    gnss = mat2GNSSData(matfile)
    im = plt.imread(imfile)
    fig, ax = plt.subplots(figsize=(10,3)) #Looks better with this size
    ax.imshow(im, extent=imExtent, aspect='auto')
    ax.plot(gts.positions[:, 0], gts.positions[:, 1])
    ax.scatter(gnss.pos[:, 0], gnss.pos[:, 1], marker="x", color="r")
    #plt.savefig("experiment.pdf", format="pdf", bbox_inches="tight")
    return fig

def main():
    import os
    from datetime import datetime

    filepath = "../data/recorded_runs/python_plots/Saved/singleLoopWithLoopClosure/"
    estimates, gts = bag2numpy(f'{filepath}simpleTunnel_IMUAndGNSSMapOptimization_LoopClosure_1.bag')
    estimatesAfterSmoothing, velocities, landmarks, biases = csv2numpy(f"{filepath}LatestRun.csv", estimates)
    beforeSmoothingEstVsGt = plotTrajectory2D(estimates, gts, skipPlt=2, xlim=[-50, 50], ylim=[-100, 400], title="Before Smoothing")
    afterSmoothingEstVsGt = plotTrajectory2D(estimatesAfterSmoothing, gts, skipPlt=2, xlim=[-50, 50], ylim=[-100, 400], title="After Smoothing")
    est_to_gt = np.zeros(estimates.times.shape, dtype=np.int)
    #print(estimates.positions.shape, estimatesAfterSmoothing.positions.shape)
    for i, est_t in enumerate(estimates.times):
        est_to_gt[i] = np.argmax(gts.times > est_t)
    est_to_gt_idxs = np.unique(est_to_gt)
    errorFig = plotErrorsOverTime(estimates, gts, est_to_gt)


    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H:%M:%S")
    figfolder = f"../data/recorded_runs/python_plots/RawRuns/{dt_string}"
    if not os.path.isdir(figfolder):
        os.makedirs(figfolder)
    
    beforeSmoothingEstVsGt.savefig(figfolder + "/beforeSmoothingEstVsGt.pdf", format='pdf', bbox_inches="tight")
    #beforeSmoothingXYWithCov.savefig(figfolder + "/beforeSmoothingXYWithCov.pdf", format='pdf', bbox_inches="tight")
    afterSmoothingEstVsGt.savefig(figfolder + "/afterSmoothingEstVsGt.pdf", format='pdf', bbox_inches="tight")
    #afterSmoothingXYWithCov.savefig(figfolder + "/afterSmoothingXYWithCov.pdf", format='pdf', bbox_inches="tight")
    errorFig.savefig(figfolder + "/absolutePositionError.pdf", format='pdf', bbox_inches="tight")
    if len(velocities) > 0:
        estimatedExtraStateEstimates = plotExtraStateEstimates(estimatesAfterSmoothing.times, velocities, biases)
        estimatedMapWithTrajectory = plotMapWithTrajectory(estimatesAfterSmoothing.positions, landmarks)
        estimatedExtraStateEstimates.savefig(figfolder + "/estimatedExtraStateEstimates.pdf", format='pdf', bbox_inches="tight")
        estimatedMapWithTrajectory.savefig(figfolder + "/estimatedMapWithTrajectory.pdf", format='pdf', bbox_inches="tight")
    
    matfile = "../data/april_2021/SimpleTunnel_Loop_10HzFreqLidar_ds.mat"
    imfile = "../simulator_matlab/straightTunnel_long.png"
    imExtent = [-10, 300, 20, -30]
    experimentFig = plotExperiment(gts, imfile, matfile, imExtent=imExtent)
    experimentFig.savefig(figfolder + "/experiment.pdf", format='pdf', bbox_inches="tight")
    plt.show()

main()
"""gnss = mat2GNSSData("../data/april_2021/SimpleTunnel_IMUGNSS_LongTime_straightPath_ds.mat")
estimates, gts = bag2numpy('../data/recorded_runs/python_plots/Saved/Over60SecRun/Imu+Gnss/simpleTunnel_IMUAndGNSSMapOptimization_1.bag')
im = plt.imread("../simulator_matlab/straightTunnel.png")
implot = plt.imshow(im, extent=[-5, 270, -15, 15])
plt.plot(gts.positions[:, 0], gts.positions[:, 1])
plt.scatter(gnss.pos[:, 0], gnss.pos[:, 1], marker="x", color="r")
plt.savefig("StraightTunnel.eps", format="eps", bbox_inches="tight")
plt.show()"""