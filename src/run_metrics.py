import numpy as np
import math
import scipy.io as sio
import scipy
import matplotlib.pyplot as plt
import matplotlib

import rospy
import rosbag

from geometry_msgs.msg import Vector3, PoseStamped, Point, Quaternion, PoseWithCovarianceStamped

class PoseData:
    def __init__(self, positions, orientations, times, covs=None):
        self.positions = positions
        self.orientations = orientations
        self.times = times
        self.covariances = covs

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
                positionArr = [position.x, -position.y, -position.z] # Rotate to correct coordinate system
                quat = msg.message.pose.orientation
                angles = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
                gt_positions.append(positionArr)
                gt_orientations.append([angles[0], angles[1], angles[2]]) # Rotate to correct coordinate system
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

estimates, gts = bag2numpy('../data/recorded_runs/simpleTunnel_IMUwithMapOptimization_1.bag')

def plotTrajectory2D(estimates:PoseData, gts:PoseData):
    fig, ax = plt.subplots(num=1, clear=True)
    ax.plot(estimates.positions[:, 1], estimates.positions[:, 0], label=r'$\hat{x}$')
    ax.plot(gts.positions[:, 1], gts.positions[:, 0], label=r'$x$')
    ax.legend()

    fig2, ax2 = plt.subplots(num=2, clear=True)
    for i, (position, orientation, cov) in enumerate(zip(estimates.positions, estimates.orientations, estimates.covariances)):
        if i%4 == 0:
            plot_cov = np.array(([[cov[1, 1], cov[1, 0]], [cov[0, 1], cov[0, 0]]]))
            plot_cov_ellipse2d(ax2, np.array([position[1], position[0]]), plot_cov, yaw = orientation[-1], edgecolor='r')
    ax2.plot(estimates.positions[:, 1], estimates.positions[:, 0], marker="x", label='XYPos')
    ax2.legend()
    
    plt.show()
plotTrajectory2D(estimates, gts)




