#!/usr/bin/env python2
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import thread
import time

import pcl
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def transform_fusion():
    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)
        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'camera_init', 'map')
        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    sor = pc_scan.make_voxel_grid_filter()
    sor.set_leaf_size(SCAN_VOXEL_SIZE * scale, SCAN_VOXEL_SIZE * scale, SCAN_VOXEL_SIZE * scale)

    # 用初始解转换到对应坐标系
    pc = np.array(sor.filter())
    pc = np.column_stack([pc, np.ones(len(pc)).reshape(-1, 1)])
    pc_in_map = (np.matmul(initial, pc.T)).T
    scan_tobe_mapped = pcl.PointCloud()
    scan_tobe_mapped.from_array(pc_in_map[:, :3].astype(np.float32))

    # 对地图降采样
    sor = pc_map.make_voxel_grid_filter()
    sor.set_leaf_size(MAP_VOXEL_SIZE * scale, MAP_VOXEL_SIZE * scale, MAP_VOXEL_SIZE * scale)
    map_down = sor.filter()

    icp = map_down.make_IterativeClosestPoint()
    converged, transformation, estimate, fitness = \
        icp.icp(scan_tobe_mapped, map_down, max_iter=10)

    # 这里要将初始解进行变换， 因为icp估计的是精确位置到初始解的delta
    return np.matmul(transformation, initial), fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(pose_estimation):
    global global_map, cur_odom

    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    # FOV_FAR>x>0 且角度小于FOV
    indices = np.where(
        (global_map_in_base_link[:, 0] > 0) &
        (global_map_in_base_link[:, 0] < FOV_FAR) &
        (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
    )
    global_map_in_FOV = pcl.PointCloud()
    global_map_in_FOV.from_array(np.squeeze(global_map_in_map[indices, :3]).astype(np.float32))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, global_map_in_FOV.to_array()[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(pose_estimation)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness < LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def initialize_global_map(pc_msg):
    global global_map
    global_map = pcl.PointCloud()
    global_map.from_array(msg_to_array(pc_msg).astype(np.float32))
    sor = global_map.make_voxel_grid_filter()
    sor.set_leaf_size(MAP_VOXEL_SIZE, MAP_VOXEL_SIZE, MAP_VOXEL_SIZE)
    global_map = sor.filter()
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = pcl.PointCloud()
    cur_scan.from_array(pc.astype(np.float32))


def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    # 全局定位的fitness预支
    LOCALIZATION_TH = 0.2

    # FOV内的最远距离
    FOV_FAR = 300

    # FOV范围(rad)
    FOV = 1.6

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # 发布定位消息
    thread.start_new_thread(transform_fusion, ())

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.loginfo('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.loginfo('Waiting for initial pose....')

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('Initialize successfully!!!!!!')
    # 开始定期全局定位
    thread.start_new_thread(thread_localization, ())
    # multiprocessing.Process(target=thread_localization, args=()).start()

    rospy.spin()
