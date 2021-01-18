#!/usr/bin/python

import numpy as np
import pandas as pd
import scipy.io as sio
import scipy

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2

def mat2pointcloud(filename):
    m = sio.loadmat(filename, simplify_cells=True)
    ptClouds = m['simple_tunnel_ds']['frontBumperLidar']['ptCloud']
    frequency = m['simple_tunnel_ds']['frontBumperLidar']['specs']['frequency']
    vn, hn, d, pcn = ptClouds.shape
    scans = ptClouds.reshape(vn*hn, d, pcn)
    return scans, frequency

def csv2pointcloud(filename, nscans):
    m = pd.read_csv(filename, sep=';')
    scans = np.hstack((m['x'], m['y'], m['z'])).reshape((-1, 3, nscans))
    #print(scans.shape)
    return scans

def mat2bag(bagname, freq=15):
    scans, freq = mat2pointcloud('datasets/SimpleTunnel_easyPath_noActor_ds.mat')

    rate = rospy.Rate(freq) #15Hz
    write_bag(scans, bagname, rate)

def csv2bag(bagname, freq=15):
    scans = csv2pointcloud('datasets/2704/towards_column.csv', 271)#811)

    rate = rospy.Rate(freq) #15Hz
    write_bag(scans, bagname, rate)

def write_bag(scans, bagname, rate:rospy.Rate):
    n = 6
    with rosbag.Bag(bagname, 'w') as bag:
        while not rospy.is_shutdown():
            try:
                cloud = PointCloud2()
                cloud.header.stamp = rospy.Time.now()
                cloud.header.frame_id = 'horizontal'
                cloud = pc2.create_cloud_xyz32(cloud.header, scans[: ,: , n])
                bag.write('/points2', cloud, cloud.header.stamp)
                n+=1
                rate.sleep()
            except:
                print('done writing')
                bag.close()
                exit()


def main():
    rospy.init_node('data2bag')
    mat2bag('simpleTunnel_easyPath.bag')
    #csv2bag('real.bag')
if __name__ == '__main__':
	main()