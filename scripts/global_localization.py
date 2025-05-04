#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

# 先导入 numpy 并补丁：新版 numpy 中 np.float 已废弃，补上兼容性修改
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import copy # 导入copy模块，用于复制对象
import _thread # 导入_thread模块，用于创建线程
import time # 导入time模块，用于时间相关操作

import open3d as o3d # 导入open3d库，用于3D数据处理
import rospy # 导入rospy库，用于ROS编程
import ros_numpy # 导入ros_numpy库，用于ROS消息和numpy数组之间的转换
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion # 导入ROS消息类型
from nav_msgs.msg import Odometry # 导入Odometry消息类型
from sensor_msgs.msg import PointCloud2 # 导入PointCloud2消息类型
import tf # 导入tf库，用于坐标变换
import tf.transformations # 导入tf.transformations模块，提供变换相关的函数

global_map = None # 定义全局地图变量，初始值为None
initialized = False # 定义初始化标志变量，初始值为False
T_map_to_odom = np.eye(4) # 定义地图到里程计的变换矩阵，初始值为单位矩阵
cur_odom = None # 定义当前里程计信息变量，初始值为None
cur_scan = None # 定义当前扫描点云变量，初始值为None


def pose_to_mat(pose_msg):
    # 将ROS位姿消息转换为4x4的变换矩阵
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position), # 将位姿消息中的位置转换为变换矩阵
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation), # 将位姿消息中的姿态转换为变换矩阵
    )


def msg_to_array(pc_msg):
    # 将ROS点云消息转换为numpy数组
    pc_array = ros_numpy.numpify(pc_msg) # 将点云消息转换为numpy数组
    pc = np.zeros([len(pc_array), 3]) # 创建一个空的numpy数组，用于存储点云数据
    pc[:, 0] = pc_array['x'] # 将x坐标赋值给numpy数组
    pc[:, 1] = pc_array['y'] # 将y坐标赋值给numpy数组
    pc[:, 2] = pc_array['z'] # 将z坐标赋值给numpy数组
    return pc # 返回numpy数组

# 使用ICP算法进行点云配准
def registration_at_scale(pc_scan, pc_map, initial, scale):
    # 在不同尺度下进行点云配准
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale), # 对点云进行体素降采样
        1.0 * scale, initial, # 设置最大对应点距离和初始变换矩阵
        o3d.pipelines.registration.TransformationEstimationPointToPoint(), # 使用点到点ICP算法
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20) # 设置ICP算法的收敛条件
    )

    return result_icp.transformation, result_icp.fitness # 返回变换矩阵和fitness score


def inverse_se3(trans):
    # 计算SE3变换矩阵的逆
    trans_inverse = np.eye(4) # 创建一个单位矩阵
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T # 将旋转矩阵转置
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3]) # 计算平移向量的逆
    return trans_inverse # 返回逆变换矩阵


def publish_point_cloud(publisher, header, pc):
    # 发布点云消息
    data = np.zeros(len(pc), dtype=[ # 创建一个numpy数组，用于存储点云数据
        ('x', np.float32), # x坐标
        ('y', np.float32), # y坐标
        ('z', np.float32), # z坐标
        ('intensity', np.float32), # 强度
    ])
    data['x'] = pc[:, 0] # 将x坐标赋值给numpy数组
    data['y'] = pc[:, 1] # 将y坐标赋值给numpy数组
    data['z'] = pc[:, 2] # 将z坐标赋值给numpy数组
    if pc.shape[1] == 4: # 如果点云数据包含强度信息
        data['intensity'] = pc[:, 3] # 将强度赋值给numpy数组
    msg = ros_numpy.msgify(PointCloud2, data) # 将numpy数组转换为ROS点云消息
    msg.header = header # 设置消息头
    publisher.publish(msg) # 发布消息


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 裁剪全局地图，只保留FOV内的点云
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom) # 获取当前里程计位姿到base_link的变换矩阵
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link) # 获取地图到base_link的变换矩阵
    T_base_link_to_map = inverse_se3(T_map_to_base_link) # 获取base_link到地图的变换矩阵

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points) # 将全局地图转换为numpy数组
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))]) # 将全局地图转换为齐次坐标
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T # 将全局地图转换到base_link坐标系下

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) & # 过滤掉距离大于FOV_FAR的点
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0) # 过滤掉角度大于FOV/2的点
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) & # 过滤掉x坐标小于0的点
            (global_map_in_base_link[:, 0] < FOV_FAR) & # 过滤掉距离大于FOV_FAR的点
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0) # 过滤掉角度大于FOV/2的点
        )
    global_map_in_FOV = o3d.geometry.PointCloud() # 创建一个新的点云对象
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3])) # 将FOV内的点云赋值给新的点云对象

    # 发布fov内点云
    header = cur_odom.header # 获取当前里程计消息头
    header.frame_id = 'map' # 设置消息头的frame_id为map
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10]) # 发布FOV内的点云

    return global_map_in_FOV # 返回FOV内的点云


def global_localization(pose_estimation):
    # 全局定位函数
    global global_map, cur_scan, cur_odom, T_map_to_odom # 声明全局变量
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......') # 打印日志信息

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan) # 复制当前扫描点云

    tic = time.time() # 记录开始时间

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom) # 裁剪全局地图，只保留FOV内的点云

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5) # 在尺度为5的情况下进行点云配准

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1) # 在尺度为1的情况下进行点云配准
    toc = time.time() # 记录结束时间
    rospy.loginfo('Time: {}'.format(toc - tic)) # 打印时间
    rospy.loginfo('') # 打印空行

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH: # 如果fitness score大于阈值
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation # 更新地图到里程计的变换矩阵

        # 发布map_to_odom
        map_to_odom = Odometry() # 创建一个Odometry消息
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom) # 从变换矩阵中提取平移向量
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom) # 从变换矩阵中提取四元数
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat)) # 设置Odometry消息的位姿
        map_to_odom.header.stamp = cur_odom.header.stamp # 设置Odometry消息的时间戳
        map_to_odom.header.frame_id = 'map' # 设置Odometry消息的frame_id
        pub_map_to_odom.publish(map_to_odom) # 发布Odometry消息
        return True # 返回True
    else:
        rospy.logwarn('Not match!!!!') # 打印警告信息
        rospy.logwarn('{}'.format(transformation)) # 打印变换矩阵
        rospy.logwarn('fitness score:{}'.format(fitness)) # 打印fitness score
        return False # 返回False


def voxel_down_sample(pcd, voxel_size):
    # 对点云进行体素降采样
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size) # 使用open3d的voxel_down_sample函数进行降采样
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size) # 使用open3d的voxel_down_sample函数进行降采样
    return pcd_down # 返回降采样后的点云


def initialize_global_map(pc_msg):
    # 初始化全局地图
    global global_map # 声明全局变量

    global_map = o3d.geometry.PointCloud() # 创建一个新的点云对象
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3]) # 将点云消息转换为numpy数组，并将点云数据赋值给新的点云对象
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE) # 对全局地图进行体素降采样
    rospy.loginfo('Global map received.') # 打印日志信息


def cb_save_cur_odom(odom_msg):
    # 回调函数，保存当前里程计信息
    global cur_odom # 声明全局变量
    cur_odom = odom_msg # 保存当前里程计信息


def cb_save_cur_scan(pc_msg):
    # 回调函数，保存当前扫描点云
    global cur_scan # 声明全局变量
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init' # 设置消息头的frame_id
    pc_msg.header.stamp = rospy.Time().now() # 设置消息头的时间戳
    pub_pc_in_map.publish(pc_msg) # 发布点云消息

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]] # 调整点云消息的fields
    pc = msg_to_array(pc_msg) # 将点云消息转换为numpy数组

    cur_scan = o3d.geometry.PointCloud() # 创建一个新的点云对象
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3]) # 将点云数据赋值给新的点云对象


def thread_localization():
    # 线程函数，定期进行全局定位
    global T_map_to_odom # 声明全局变量
    while True: # 循环
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION) # 休眠一段时间
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom) # 进行全局定位


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.4 # 地图体素大小
    SCAN_VOXEL_SIZE = 0.1 # 扫描点云体素大小

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5 # 全局定位频率

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.8 # 全局定位阈值

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28319 # 视场角

    # The farthest distance(meters) within FOV
    FOV_FAR = 300 # 视场角最远距离

    rospy.init_node('fast_lio_localization') # 初始化ROS节点
    rospy.loginfo('Localization Node Inited...') # 打印日志信息

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1) # 创建一个发布者，用于发布当前扫描点云在地图坐标系下的点云
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1) # 创建一个发布者，用于发布子地图
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1) # 创建一个发布者，用于发布地图到里程计的变换

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1) # 创建一个订阅者，用于订阅注册后的点云
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1) # 创建一个订阅者，用于订阅里程计信息

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......') # 打印警告信息
    initialize_global_map(rospy.wait_for_message('map', PointCloud2)) # 初始化全局地图

    # 初始化
    while not initialized: # 循环，直到初始化成功
        rospy.logwarn('Waiting for initial pose....') # 打印警告信息

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped) # 等待初始位姿消息
        initial_pose = pose_to_mat(pose_msg) # 将初始位姿消息转换为变换矩阵
        if cur_scan: # 如果当前扫描点云已经接收到
            initialized = global_localization(initial_pose) # 进行全局定位
        else:
            rospy.logwarn('First scan not received!!!!!') # 打印警告信息

    rospy.loginfo('') # 打印空行
    rospy.loginfo('Initialize successfully!!!!!!') # 打印日志信息
    rospy.loginfo('') # 打印空行
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ()) # 创建一个新的线程，用于定期进行全局定位

    rospy.spin() # 循环，直到ROS节点关闭
