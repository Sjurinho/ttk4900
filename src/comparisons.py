from run_metrics import PoseData, GNSSData, mat2GNSSData, csv2numpy, bag2numpy

import numpy as np
import scipy.stats

import os
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib as mpl


mpl.style.use('seaborn')
plt.rcParams["lines.markeredgewidth"] = 0.5

#basePath = "../data/recorded_runs/python_plots/Saved/newestSensorComparisons"
basePath = "../data/recorded_runs/python_plots/Saved/singleLoopWithLoopClosure"

experiment = basePath.split("/")[-1]

#folders = [f"{basePath}/lidar", f"{basePath}/lidar+imu", f"{basePath}/lidar+imu+gnss"]
#bagfiles = [folder+"/rawEstimates.bag" for folder in folders]
#csvfiles = [folder+"/LatestRun.csv" for folder in folders]
folders = [f"{basePath}/noLoopClosure", f"{basePath}/11"]
bagfiles = [folder+"/rawEstimates.bag" for folder in folders]
csvfiles = [folder+"/LatestRun.csv" for folder in folders]
print(bagfiles, csvfiles)
#labels = ["LiDAR", "LiDAR+IMU", "LiDAR+IMU+GNSS"]
labels = ["Without Loop Closure", "With Loop Closure"]
def angleError(alpha, beta):
    phi = np.abs(beta-alpha) % 360
    distance = 360-phi if phi > 180 else phi
    return distance
def angleErrorRad(alpha, beta):
    phi = np.abs(beta-alpha) % (2*np.pi)
    distance = 2*np.pi-phi if phi > np.pi else phi
    return distance

estimatesBS = []
estimatesAS = []
matchesBS = []
matchesAS = []
gt_list = []
for (bagfile, csvfile) in zip(bagfiles, csvfiles):
    estimatesBeforeSmoothing, gts = bag2numpy(bagfile)
    estimatesAfterSmoothing, velocities, landmarks, biases = csv2numpy(csvfile, estimatesBeforeSmoothing)
    gt_list.append(gts)
    estimatesBS.append(estimatesBeforeSmoothing)
    estimatesAS.append(estimatesAfterSmoothing)
    est_to_gt_before_smoothing = np.zeros(estimatesBeforeSmoothing.times.shape, dtype=np.int)

    for i, est_t in enumerate(estimatesBeforeSmoothing.times):
        est_to_gt_before_smoothing[i] = np.argmax(gts.times > est_t)
        if est_to_gt_before_smoothing[i] == 0 and i != 0:
            est_to_gt_before_smoothing[i] = -1
    
    est_to_gt_after_smoothing = np.zeros(estimatesAfterSmoothing.times.shape, dtype=np.int)

    for i, est_t in enumerate(estimatesAfterSmoothing.times):
        est_to_gt_after_smoothing[i] = np.argmax(gts.times > est_t)
        if est_to_gt_after_smoothing[i] == 0 and i != 0:
            est_to_gt_after_smoothing[i] = -1
    matchesBS.append(est_to_gt_before_smoothing)
    matchesAS.append(est_to_gt_after_smoothing)

greyArea = []
func = np.vectorize(angleError)
figErrors, axsErrors = plt.subplots(4, 1, clear=True, figsize=(10,10), sharex=True)
figErrors.suptitle(" Absolute Errors", fontsize=25)
confprob = 0.95
CI3 = np.array(scipy.stats.chi2.interval(confprob, 3)).reshape((2,1))
CI2 = np.array(scipy.stats.chi2.interval(confprob, 2)).reshape((2,1))
CI1 = np.array(scipy.stats.chi2.interval(confprob, 1)).reshape((2,1))
figNees, axNees = plt.subplots(3, 1, clear=True, figsize=(10,10), sharex=True)
RMSEBS_pos = []
RMSEBS_ori = []
RMSEBS_x = []
RMSEBS_y = []
ANEESesBS_pos = []
ANEESesBS_ori = []
ANEESesBS_total = []
RMSEAS_pos = []
RMSEAS_ori = []
RMSEAS_x = []
RMSEAS_y = []
ANEESesAS_pos = []
ANEESesAS_ori = []
ANEESesAS_total = []
print(len(matchesBS))
for i, (estimatesBeforeSmoothing, estimatesAfterSmoothing, matchesBeforeSmoothing, matchesAfterSmoothing, gts) in enumerate(zip(estimatesBS, estimatesAS, matchesBS, matchesAS, gt_list)):
    # Before smoothing
    axsErrors[0].plot(gts.times[matchesBeforeSmoothing], np.linalg.norm(estimatesBeforeSmoothing.positions - gts.positions[matchesBeforeSmoothing], ord=2, axis=1), color="C"+str(i), label=labels[i] + " Before Smoothing")
    if len(greyArea) > 0:
        axsErrors[0].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        axsErrors[0].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axsErrors[0].set_xlim([0, gts.times[-1]])

    axsErrors[0].set_title("XY Position Error")
    axsErrors[0].set_ylabel("Error [m]")
    axsErrors[3].plot(estimatesBeforeSmoothing.times, gts.positions[matchesBeforeSmoothing, 1] - estimatesBeforeSmoothing.positions[:, 1], color="C"+str(i), label=labels[i] + "Before Smoothing")
    axsErrors[2].plot(estimatesBeforeSmoothing.times, gts.positions[matchesBeforeSmoothing, 0] - estimatesBeforeSmoothing.positions[:, 0],color="C"+str(i), label=labels[i] + "Before Smoothing")
    axsErrors[1].plot(gts.times[matchesBeforeSmoothing], func(np.rad2deg(estimatesBeforeSmoothing.orientations[:, -1]), np.rad2deg(gts.orientations[matchesBeforeSmoothing, -1])), color="C"+str(i), label=labels[i] + " Before Smoothing")
    if len(greyArea) > 0:
        axsErrors[1].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
        axsErrors[1].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axsErrors[1].set_xlim([0, gts.times[-1]])
    axsErrors[1].set_ylabel("Error [deg]")
    axsErrors[1].set_title("Heading Error")

    # After smoothing
    axsErrors[0].plot(gts.times[matchesAfterSmoothing], np.linalg.norm(estimatesAfterSmoothing.positions - gts.positions[matchesAfterSmoothing], ord=2, axis=1), linestyle="dashed", color="C"+str(i), label=labels[i] + " After Smoothing")
    axsErrors[1].plot(gts.times[matchesAfterSmoothing], func(np.rad2deg(estimatesAfterSmoothing.orientations[:, -1]), np.rad2deg(gts.orientations[matchesAfterSmoothing, -1])), linestyle="dashed", color="C"+str(i), label=labels[i] + " After Smoothing")
    axsErrors[3].plot(gts.times[matchesAfterSmoothing], gts.positions[matchesAfterSmoothing, 1] - estimatesAfterSmoothing.positions[:, 1], color="C"+str(i), label=labels[i] + "After Smoothing", linestyle="dashed")
    axsErrors[2].plot(estimatesAfterSmoothing.times, gts.positions[matchesAfterSmoothing, 0] - estimatesAfterSmoothing.positions[:, 0], color="C"+str(i), label=labels[i] + "After Smoothing", linestyle="dashed")
    axsErrors[0].legend()
    axsErrors[1].legend()
    axsErrors[2].legend()
    axsErrors[3].legend()

    axsErrors[2].set_title("X Position Error")
    axsErrors[2].set_ylabel("Error [m]")
    axsErrors[3].set_title("Y Position Error")
    axsErrors[3].set_ylabel("Error [m]")

    plt.xlabel("Time [s]")

    #fig.suptitle("Planar NEES Over Time")
    NEESTotalBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    NEESPosBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    NEESYawBeforeSmoothing = np.zeros(estimatesBeforeSmoothing.positions.shape[0])
    errors_pos = np.zeros((estimatesAfterSmoothing.positions.shape[0], 2))
    errors_ori = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    for k, (est_pos, est_ori, est_cov) in enumerate(zip(estimatesBeforeSmoothing.positions, estimatesBeforeSmoothing.orientations, estimatesBeforeSmoothing.covariances)):
        gt_pos = gts.positions[matchesBeforeSmoothing[k]]
        gt_ori = gts.orientations[matchesBeforeSmoothing[k]]

        # XY
        e_pos = est_pos[:2] - gt_pos[:2]
        P = np.array(([[est_cov[1, 1], est_cov[1, 0]], [est_cov[0, 1], est_cov[0, 0]]]))
        P_inv = np.linalg.inv(P)
        NEESPosBeforeSmoothing[k] = e_pos.T @ P_inv @ e_pos

        # Yaw
        e_yaw = angleErrorRad(est_ori[-1], gt_ori[-1])
        cov_yaw = est_cov[-1,-1]
        NEESYawBeforeSmoothing[k] = e_yaw*e_yaw / cov_yaw

        #Total
        P = np.array(([[est_cov[1,1], est_cov[1, 0], est_cov[1, -1]], [est_cov[0, 1], est_cov[0, 0], est_cov[0, -1]], [est_cov[-1, 1], est_cov[-1, 0], est_cov[-1, -1]]]))
        P_inv = np.linalg.inv(P)
        e_total = np.array([*e_pos, e_yaw])
        NEESTotalBeforeSmoothing[k] = e_total.T @ P_inv @ e_total

        errors_pos[k] = e_pos
        errors_ori[k] = e_yaw

    ANEESesBS_pos.append(np.mean(NEESPosBeforeSmoothing))
    ANEESesBS_ori.append(np.mean(NEESYawBeforeSmoothing))
    ANEESesBS_total.append(np.mean(NEESTotalBeforeSmoothing))

    RMSEBS_pos.append(np.sqrt(np.mean(np.linalg.norm(errors_pos, axis=1)**2)))
    RMSEBS_x.append(np.sqrt(np.mean(errors_pos[:, 0]**2)))
    RMSEBS_y.append(np.sqrt(np.mean(errors_pos[:, 1]**2)))
    RMSEBS_ori.append(np.sqrt(np.mean(errors_ori**2)))

    axNees[0].plot(estimatesBeforeSmoothing.times, NEESPosBeforeSmoothing, color="C"+str(i), label=labels[i] + " Before Smoothing")
    axNees[1].plot(estimatesBeforeSmoothing.times, NEESYawBeforeSmoothing, color="C"+str(i), label=labels[i] + " Before Smoothing")
    axNees[2].plot(estimatesBeforeSmoothing.times, NEESTotalBeforeSmoothing, color="C"+str(i), label=labels[i] + " Before Smoothing")

    NEESTotalAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    NEESPosAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    NEESYawAfterSmoothing = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    errors_pos = np.zeros((estimatesAfterSmoothing.positions.shape[0], 2))
    errors_ori = np.zeros(estimatesAfterSmoothing.positions.shape[0])
    for k, (est_pos, est_ori, est_cov) in enumerate(zip(estimatesAfterSmoothing.positions, estimatesAfterSmoothing.orientations, estimatesAfterSmoothing.covariances)):
        gt_pos = gts.positions[matchesAfterSmoothing[k]]
        gt_ori = gts.orientations[matchesAfterSmoothing[k]]

        # XY
        e_pos = est_pos[:2] - gt_pos[:2]
        P = np.array(([[est_cov[1, 1], est_cov[1, 0]], [est_cov[0, 1], est_cov[0, 0]]]))
        P_inv = np.linalg.inv(P)
        NEESPosAfterSmoothing[k] = e_pos.T @ P_inv @ e_pos

        # Yaw
        e_yaw = angleErrorRad(est_ori[-1], gt_ori[-1])
        cov_yaw = est_cov[-1,-1]
        NEESYawAfterSmoothing[k] = e_yaw*e_yaw / cov_yaw
        #print(e_yaw)

        # Total
        P = np.array(([[est_cov[1,1], est_cov[1, 0], est_cov[1, -1]], [est_cov[0, 1], est_cov[0, 0], est_cov[0, -1]], [est_cov[-1, 1], est_cov[-1, 0], est_cov[-1, -1]]]))
        P_inv = np.linalg.inv(P)
        e_total = np.array([*e_pos, e_yaw])
        NEESTotalAfterSmoothing[k] = e_total.T @ P_inv @ e_total

        errors_pos[k] = e_pos
        errors_ori[k] = e_yaw

    ANEESesAS_pos.append(np.mean(NEESPosAfterSmoothing))
    ANEESesAS_ori.append(np.mean(NEESYawAfterSmoothing))
    ANEESesAS_total.append(np.mean(NEESTotalAfterSmoothing))

    RMSEAS_pos.append(np.sqrt(np.mean(np.linalg.norm(errors_pos, axis=1)**2)))
    RMSEAS_x.append(np.sqrt(np.mean(errors_pos[:, 0]**2)))
    RMSEAS_y.append(np.sqrt(np.mean(errors_pos[:, 1]**2)))
    RMSEAS_ori.append(np.sqrt(np.mean(errors_ori**2)))

    axNees[0].set_title(f"XY Position NEESes")
    axNees[1].set_title(f"Heading NEESes")
    axNees[2].set_title(f"Total Planar NEESes")


    axNees[0].plot(estimatesAfterSmoothing.times, NEESPosAfterSmoothing, linestyle="dashed", color="C"+str(i), label=labels[i] + " After Smoothing")
    axNees[1].plot(estimatesAfterSmoothing.times, NEESYawAfterSmoothing, linestyle="dashed", color="C"+str(i), label=labels[i] + " After Smoothing")
    axNees[2].plot(estimatesAfterSmoothing.times, NEESTotalAfterSmoothing, linestyle="dashed", color="C"+str(i), label=labels[i] + " After Smoothing")


axNees[0].plot(estimatesAS[-1].times, np.repeat(CI2[0], estimatesAS[-1].times.shape[0]), color="C3", label="Lower Bound")
axNees[0].plot(estimatesAS[-1].times, np.repeat(CI2[1], estimatesAS[-1].times.shape[0]), color="C4", label="Upper Bound")
axNees[1].plot(estimatesAS[-1].times, np.repeat(CI1[0], estimatesAS[-1].times.shape[0]), color="C3", label="Lower Bound")
axNees[1].plot(estimatesAS[-1].times, np.repeat(CI1[1], estimatesAS[-1].times.shape[0]), color="C4", label="Upper Bound")
axNees[2].plot(estimatesAS[-1].times, np.repeat(CI3[0], estimatesAS[-1].times.shape[0]), color="C3", label="Lower Bound")
axNees[2].plot(estimatesAS[-1].times, np.repeat(CI3[1], estimatesAS[-1].times.shape[0]), color="C4", label="Upper Bound")
if len(greyArea) > 0:
    #axNees.axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
    #axNees.axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axNees[0].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
    axNees[0].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axNees[1].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
    axNees[1].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
    axNees[2].axvspan(greyArea[0], greyArea[1], facecolor='grey', alpha=0.3)
    axNees[2].axvspan(greyArea[2], greyArea[3], facecolor='grey', alpha=0.3)
#axNees.set_xlim([0, gts.times[-1]])
#axNees.legend()
axNees[0].set_xlim([0, gts.times[-1]])
axNees[0].legend(loc='upper left')
axNees[1].set_xlim([0, gts.times[-1]])
axNees[1].legend(loc='upper left')
axNees[2].set_xlim([0, gts.times[-1]])
axNees[2].legend(loc='upper left')

figfolder = f"../data/recorded_runs/python_plots/comparisons/"
if not os.path.isdir(figfolder):
    os.makedirs(figfolder)

saveConfig = {
    "format":"pdf", 
    "bbox_inches":"tight", 
    "dpi":1200
    }

figErrors.savefig(
    figfolder + f"/{experiment}_AbsoluteErrors.pdf", 
    **saveConfig
)
figNees.savefig(
    figfolder + f"/{experiment}_PlanarNeeses.pdf", 
    **saveConfig
    )
NsBS = [arr.positions.shape[0] for arr in estimatesBS]
NsAS = [arr.positions.shape[0] for arr in estimatesAS]

CI1NBS = [np.array(scipy.stats.chi2.interval(confprob, N)) / N for N in NsBS]
CI1NAS = [np.array(scipy.stats.chi2.interval(confprob, N)) / N for N in NsAS]

CI2NBS = [np.array(scipy.stats.chi2.interval(confprob, 2*N)) / N for N in NsBS]
CI2NAS = [np.array(scipy.stats.chi2.interval(confprob, 2*N)) / N for N in NsAS]

CI3NBS = [np.array(scipy.stats.chi2.interval(confprob, 3*N)) / N for N in NsBS]
CI3NAS = [np.array(scipy.stats.chi2.interval(confprob, 3*N)) / N for N in NsAS]

print(f"""
    --------------RMSEs-----------------
    RMSEBS_x: {RMSEBS_x}
    RMSEBS_y: {RMSEBS_y}
    RMSEBS_pos: {RMSEBS_pos}
    RMSEBS_ori: {np.rad2deg(RMSEBS_ori)}

    RMSEAS_x: {RMSEAS_x}
    RMSEAS_y: {RMSEAS_y}
    RMSEAS_pos: {RMSEAS_pos}
    RMSEAS_ori: {np.rad2deg(RMSEAS_ori)}

    --------------ANEESes-----------------
    ANEESesBS_pos: {ANEESesBS_pos} \t conf.int: {CI2NBS}
    ANEESesBS_ori: {ANEESesBS_ori} \t conf.int: {CI1NBS}
    ANEESesBS_total: {ANEESesBS_total} \t conf.int: {CI3NBS}

    ANEESesAS_pos: {ANEESesAS_pos} \t conf.int: {CI2NAS}
    ANEESesAS_ori: {ANEESesAS_ori} \t conf.int: {CI1NAS}
    ANEESesAS_total: {ANEESesAS_total} \t conf.int: {CI3NAS}
    """)
plt.show()