# coding=utf8
# !/usr/bin/env python2
from __future__ import print_function, division, absolute_import

import copy
import thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations
import multiprocessing

global_map = o3d.geometry.PointCloud()
initialized = False
T_map_to_odom = np.eye(4)
cur_scan = o3d.geometry.PointCloud()


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def publish_tf():
    br = tf.TransformBroadcaster()
    while True:
        rospy.sleep(0.01)
        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'camera_init', 'map')


def global_localization(odometry_pose):
    global global_map, cur_scan, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('scan to map matching......')

    # TODO 这里注意线程安全
    # 估计法线
    scan_tobe_mapped = copy.copy(cur_scan)
    scan_tobe_mapped.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    scan_tobe_mapped = scan_tobe_mapped.voxel_down_sample(SCAN_VOXEL_SIZE)

    tic = time.time()
    # 粗配准
    icp_coarse = o3d.registration.registration_icp(
        scan_tobe_mapped.voxel_down_sample(SCAN_VOXEL_SIZE * 5), global_map.voxel_down_sample(MAP_VOXEL_SIZE * 5),
        MAP_VOXEL_SIZE * 5, odometry_pose,
        o3d.registration.TransformationEstimationPointToPoint())
    # 配准
    icp_fine = o3d.registration.registration_icp(
        scan_tobe_mapped, global_map,
        MAP_VOXEL_SIZE, icp_coarse.transformation,
        o3d.registration.TransformationEstimationPointToPoint())
    print(icp_fine)

    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))

    # 当全局定位成功时才更新map2odom
    if icp_fine.fitness > 0.9:
        T_map_to_odom = icp_fine.transformation
        return True
    else:
        rospy.logwarn('Not match!!!!')
        return False


def initialize_global_map(pc_msg):
    global global_map
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg))
    global_map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    global_map.voxel_down_sample(MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(2)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # 发布定位消息
    thread.start_new_thread(publish_tf, ())

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    # rospy.Subscriber('/livox/lidar/pc2', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)

    # 初始化全局地图
    rospy.loginfo('Waiting for global map')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.loginfo('Waiting for initial pose')

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = np.matmul(
            tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
            tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
        )
        initialized = global_localization(initial_pose)

    rospy.loginfo('Initialized successfully!!!!!!')
    # 开始定期全局定位
    thread.start_new_thread(thread_localization, ())
    # multiprocessing.Process(target=thread_localization, args=()).start()

    rospy.spin()
