#!/usr/bin/env python

import rosbag
import rospy
import csv
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
import time
import os

# rospy.init_node('extract_and_plot_localization') 

# 设置matplotlib字体大小为五号（约12pt）
plt.rcParams.update({'font.size': 12})

if __name__ == '__main__':
    bag_file = 'trajectory.bag'  # 替换为您的 rosbag 文件名
    tum_file = 'src/fast_lio_localization/path_evaluation/localization.tum'  # 输出 TUM 文件名
    image_file = 'src/fast_lio_localization/path_evaluation/localization.png'

    # 新增：定义允许的话题名称变体（根据实际情况调整）
    TARGET_TOPICS = [
        '/localization',    # 标准名称
        'localization',     # 可能缺少斜杠
        '/Localization',    # 可能大小写不一致
        '/localization/odom' # 可能子话题
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
            
            # 提取数据
            trajectory_data = []
            for topic, msg, t in bag.read_messages(topics=matched_topics):
                # 添加类型检查
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    # 提取位置
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    
                    # 提取四元数
                    qx = msg.pose.pose.orientation.x
                    qy = msg.pose.pose.orientation.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w
                    
                    # 保存为TUM格式: timestamp x y z qx qy qz qw
                    trajectory_data.append([t.to_sec(), x, y, z, qx, qy, qz, qw])
            
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
            data = np.loadtxt(tum_file)
            x = data[:, 1]
            y = data[:, 2]
            z = data[:, 3]

            # 创建左右分栏的图形
            fig = plt.figure(figsize=(16, 8))
            
            # 左侧2D图
            ax1 = fig.add_subplot(121)
            ax1.plot(x, y, marker='o', linestyle='-', markersize=2)
            ax1.set_xlabel('X (m)', fontsize=12)
            ax1.set_ylabel('Y (m)', fontsize=12)
            ax1.set_title('Localization Trajectory (2D)', fontsize=12)
            ax1.tick_params(labelsize=12)
            ax1.grid(True)
            ax1.axis('equal')  # 保持XY轴比例一致
            
            # 右侧3D图
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.plot(x, y, z, marker='o', linestyle='-', markersize=2)
            ax2.set_xlabel('X (m)', fontsize=12)
            ax2.set_ylabel('Y (m)', fontsize=12)
            ax2.set_zlabel('Z (m)', fontsize=12)
            ax2.set_title('Localization Trajectory (3D)', fontsize=12)
            ax2.tick_params(labelsize=12)
            ax2.grid(True)
            # ax2.view_init(elev=30, azim=-45)  # 将azim从45度逆时针旋转90度到135度
            
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
