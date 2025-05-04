#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
import os
from mpl_toolkits.mplot3d import Axes3D

# rospy.init_node('extract_and_plot_localization') 

if __name__ == '__main__':
    bag_file = 'trajectory.bag'  # 替换为您的 rosbag 文件名
    tum_file = 'src/fast_lio_localization/path_evaluation/odometry.tum'  # 输出 TUM 文件名
    image_file = 'src/fast_lio_localization/path_evaluation/odometry.png'

    # 新增：定义允许的话题名称变体（根据实际情况调整）
    TARGET_TOPICS = [
        '/Odometry',    # 标准名称
        'Odometry',     # 可能缺少斜杠
        '/Odometry',    # 可能大小写不一致
        '/Odometry/odom' # 可能子话题
    ]

    try:
        # 检查文件存在性
        if not os.path.exists(bag_file):
            raise FileNotFoundError(f"Bag文件 {bag_file} 不存在")

        # 打开bag文件并扫描话题
        with rosbag.Bag(bag_file, 'r') as bag:
            # 获取所有话题信息
            topics = bag.get_type_and_topic_info().topics
            print("="*50)
            print("Bag文件诊断信息:")
            print(f"- 持续时间: {bag.get_end_time() - bag.get_start_time():.2f} 秒")
            print(f"- 包含话题:")
            for topic_name, topic_info in topics.items():
                print(f"  {topic_name} ({topic_info.msg_type}): {topic_info.message_count} 条消息")
            print("="*50)

            # 自动匹配目标话题
            matched_topics = [t for t in TARGET_TOPICS if t in topics]
            if not matched_topics:
                raise ValueError("未找到匹配的目标话题")
            
            print(f"使用话题: {matched_topics[0]}")
            
            # 提取数据 (TUM格式: timestamp tx ty tz qx qy qz qw)
            trajectory_data = []
            for topic, msg, t in bag.read_messages(topics=matched_topics):
                # 添加类型检查
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    timestamp = t.to_sec()
                    tx = msg.pose.pose.position.x
                    ty = msg.pose.pose.position.y
                    tz = msg.pose.pose.position.z
                    qx = msg.pose.pose.orientation.x
                    qy = msg.pose.pose.orientation.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w
                    trajectory_data.append([timestamp, tx, ty, tz, qx, qy, qz, qw])
            
            print(f"成功提取 {len(trajectory_data)} 条轨迹数据")

            # 保存TUM格式文件
            with open(tum_file, 'w') as f:
                # TUM格式不需要标题行，直接写入数据
                for data in trajectory_data:
                    # 格式化为TUM标准格式：timestamp x y z qx qy qz qw
                    line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*data)
                    f.write(line)
                print(f"数据已保存为TUM格式至 {tum_file}")

            # ------------------ 可视化部分 ------------------
            # 从TUM文件加载数据
            data = np.loadtxt(tum_file)
            # 提取X坐标（第2列）
            x = data[:, 1]
            # 提取Y坐标（第3列）
            y = data[:, 2]
            # 提取Z坐标（第4列）
            z = data[:, 3]

            # 创建一个包含两个子图的图形：2D和3D视图
            fig = plt.figure(figsize=(16, 8))
            
            # 2D轨迹图 (左侧)
            ax1 = fig.add_subplot(121)
            ax1.plot(x, y, marker='o', linestyle='-', markersize=2)
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_title('Odometry Trajectory (2D)')
            ax1.grid(True)
            ax1.axis('equal')
            
            # 3D轨迹图 (右侧)
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.plot(x, y, z, marker='o', linestyle='-', markersize=2)
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_zlabel('Z (m)')
            ax2.set_title('Odometry Trajectory (3D)')
            ax2.grid(True)
            
            plt.tight_layout()
            
            # 保存图片
            
            plt.savefig(image_file)
            print(f"轨迹图片已保存至 {image_file}")
            
            # 显示图形
            plt.show()
            

    except FileNotFoundError as e:
        print(f"错误: {str(e)}")
    except Exception as e:
        print(f"处理数据时发生错误: {str(e)}")
