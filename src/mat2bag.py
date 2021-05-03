#!/usr/bin/python

import numpy as np
import pandas as pd
import scipy.io as sio
import scipy

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import Vector3, PoseStamped, Point, Quaternion

        
class ImuData:
    def __init__(self, accelerometerData, gyroscopeData, time):
        self.accelerometer = accelerometerData
        self.gyroscope = gyroscopeData
        self.time = time

class GTData:
    def __init__(self, gtPos, gtOri, time):
        self.pos = gtPos
        self.ori = gtOri
        self.time = time

class GNSSData:
    def __init__(self, positions, time):
        self.pos = positions
        self.time = time

def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def mat2pointcloud(filename):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    ptClouds = m['simple_tunnel_ds']['frontBumperLidar']['ptCloud']
    times = m['simple_tunnel_ds']['frontBumperLidar']['time']
    frequency = m['simple_tunnel_ds']['frontBumperLidar']['specs']['frequency']
    vn, hn, d, pcn = ptClouds.shape
    scans = ptClouds.reshape(vn*hn, d, pcn)
    return scans, times, frequency

def csv2pointcloud(filename, nscans):
    m = pd.read_csv(filename, sep=';')
    scans = np.hstack((m['x'], m['y'], m['z'])).reshape((-1, 3, nscans))
    #print(scans.shape)
    return scans

def mat2ImuData(filename):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    acc = m['simple_tunnel_ds']['imu']['accelerometer'].T
    gyro = m['simple_tunnel_ds']['imu']['gyroscope'].T
    time = m['simple_tunnel_ds']['imu']['time'].T
    return ImuData(acc, gyro, time)

def mat2GTData(filename):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    Pos = m['simple_tunnel_ds']['Pgt']['signals']['values'].T
    Ori = m['simple_tunnel_ds']['Thetagt']['signals']['values'].T
    time = m['simple_tunnel_ds']['Pgt']['time'].T
    return GTData(Pos, Ori, time)

def mat2GNSSData(filename):
    try:
        m = sio.loadmat(filename, simplify_cells=True)
    except:
        import mat73
        m = mat73.loadmat(filename)
    gnss_pos = m['simple_tunnel_ds']['gnss']['measurements']
    gnss_time = m['simple_tunnel_ds']['gnss']['times']
    return GNSSData(gnss_pos, gnss_time)

def mat2bag(bagname, freq=15):
    filename = 'SimpleTunnel_BigLoop_WithTrueAccAndAngularRate.mat'
    scans, times, freq = mat2pointcloud(filename)
    imuData = mat2ImuData(filename)
    gtData = mat2GTData(filename)
    gnssData = mat2GNSSData(filename)

    rate = rospy.Rate(freq) #15Hz
    write_bag(scans, times, bagname, rate, useImu=True, imuData=imuData, useGroundTruth=True, groundTruthData=gtData, useGNSS=True, GNSSData=gnssData)

def csv2bag(bagname, freq=15):
    scans = csv2pointcloud('datasets/2704/towards_column.csv', 271)#811)

    rate = rospy.Rate(freq) #15Hz
    #write_bag(scans, bagname, rate)

def write_bag(scans, times, bagname, rate:rospy.Rate, useImu=False, imuData:ImuData=None, useGroundTruth=False, groundTruthData:GTData=None, useGNSS=True, GNSSData: GNSSData=None):
    n_start = 0
    n_scans = scans.shape[2]
    n_imu = imuData.time.shape[0] if imuData != None else 0
    n_gnss = GNSSData.time.shape[0] if GNSSData != None else 0
    n_gt = groundTruthData.time.shape[0] if groundTruthData != None else 0
    print(f'n_scans: {n_scans}, n_imu: {n_imu}, n_gt: {n_gt}, n_gnss: {n_gnss}')
    with rosbag.Bag(bagname, 'w') as bag:
        for n in range(n_start, n_scans):
            try:
                cloud = PointCloud2()
                cloud.header.stamp = rospy.Time.from_sec(times[n])
                cloud.header.frame_id = 'lidar'
                cloud = pc2.create_cloud_xyz32(cloud.header, scans[: ,: , n])
                bag.write('/points2', cloud, cloud.header.stamp)
            except:
                bag.close()
                rospy.signal_shutdown("Bag error during writing pointcloud")
                exit()
        if useImu:
            for n in range(n_start, n_imu):
                try:
                    imuMsg = Imu()
                    linAcc = Vector3()
                    angVel = Vector3()
                    
                    linAcc.x = imuData.accelerometer[n, 0]
                    linAcc.y = imuData.accelerometer[n, 1]
                    linAcc.z = imuData.accelerometer[n, 2]

                    angVel.x = imuData.gyroscope[n, 0]
                    angVel.y = imuData.gyroscope[n, 1]
                    angVel.z = imuData.gyroscope[n, 2]
                    
                    imuMsg.header.stamp = rospy.Time.from_sec(imuData.time[n])
                    imuMsg.header.frame_id = 'body'
                    imuMsg.angular_velocity = angVel
                    imuMsg.linear_acceleration = linAcc
                    
                    bag.write('/imu', imuMsg, imuMsg.header.stamp)
                except:
                    bag.close()
                    rospy.signal_shutdown("Bag error during writing imu")
                    exit()
        if useGNSS:
            for n in range(n_start, n_gnss):
                try:
                    gnssMsg = PoseStamped()
                    position = Point()
                    
                    position.x = GNSSData.pos[n, 0]
                    position.y = GNSSData.pos[n, 1]
                    position.z = GNSSData.pos[n, 2]
                    
                    gnssMsg.header.stamp = rospy.Time.from_sec(GNSSData.time[n])
                    gnssMsg.header.frame_id = 'world'
                    gnssMsg.pose.position = position
                    
                    bag.write('/gnss', gnssMsg, gnssMsg.header.stamp)
                except:
                    bag.close()
                    rospy.signal_shutdown("Bag error during writing imu")
                    exit()

        if useGroundTruth:
            for n in range(n_start, n_gt):
                try:
                    poseMsg = PoseStamped()
                    position = Point()
                    orientation = Quaternion()

                    orientationList = groundTruthData.ori[n, :]
                    position.x = groundTruthData.pos[n, 0]
                    position.y = groundTruthData.pos[n, 1]
                    position.z = groundTruthData.pos[n, 2]

                    orientationQuatList = euler_to_quaternion(*orientationList)
                    orientation.x = orientationQuatList[0]
                    orientation.y = orientationQuatList[1]
                    orientation.z = orientationQuatList[2]
                    orientation.w = orientationQuatList[3]

                    poseMsg.header.stamp = rospy.Time.from_sec(groundTruthData.time[n])
                    poseMsg.header.frame_id = 'map'
                    poseMsg.pose.position = position
                    poseMsg.pose.orientation = orientation

                    bag.write('/ground_truth', poseMsg, poseMsg.header.stamp)
                except:
                    bag.close()
                    rospy.signal_shutdown("Bag error during writing ground truth")
                    exit()

        bag.close()
        rospy.signal_shutdown("Done writing")
        exit()

def main():
    rospy.init_node('data2bag')
    mat2bag('SimpleTunnel_BigLoop2_5Hz.bag')
    #csv2bag('real.bag')
if __name__ == '__main__':
	main()