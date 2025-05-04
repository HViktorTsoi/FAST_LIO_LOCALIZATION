#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import os
from mpl_toolkits.mplot3d import Axes3D

# 设置matplotlib字体大小为五号
plt.rcParams.update({'font.size': 26})
# 设置matplotlib不使用图形界面后端
plt.switch_backend('Agg')

def plot_trajectory(x_data, y_data, z_data, title_prefix, color='r', fig_size=(18, 8), 
                    labelpad=20, elev=30, azim=-45, output_file=None):
    """
    绘制轨迹的2D和3D图
    
    参数:
        x_data, y_data, z_data: 轨迹坐标数据
        title_prefix: 图表标题前缀，如 'Odometry' 或 'Localization'
        color: 轨迹线条颜色
        fig_size: 图形尺寸
        labelpad: 轴标签与轴的间距
        elev, azim: 3D图的视角参数
        output_file: 输出图片文件路径，若为None则不保存
    """
    fig = plt.figure(figsize=fig_size)
    
    # 左侧：2D轨迹
    ax1 = fig.add_subplot(121)
    ax1.plot(x_data, y_data, marker='o', linestyle='-', markersize=2, color=color)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'{title_prefix} Trajectory (2D)')
    ax1.tick_params()
    ax1.grid(True)
    ax1.axis('equal')  # 保持XY轴比例一致
    
    # 右侧：3D轨迹
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot(x_data, y_data, z_data, marker='o', linestyle='-', markersize=2, color=color)
    ax2.set_xlabel('X (m)', labelpad=labelpad)
    ax2.set_ylabel('Y (m)', labelpad=labelpad)
    ax2.set_zlabel('Z (m)', labelpad=labelpad)
    ax2.set_title(f'{title_prefix} Trajectory (3D)')
    ax2.tick_params()
    ax2.grid(True)
    # 设置3D图的视角
    ax2.view_init(elev=elev, azim=azim)
    
    plt.tight_layout()
    
    # 如果提供了输出文件路径，则保存图像
    if output_file:
        plt.savefig(output_file)
        print(f"{title_prefix}轨迹图片已保存至 {output_file}")
    
    plt.close(fig)  # 关闭图形，避免显示
    return fig

if __name__ == '__main__':
    # 获取当前脚本所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"当前脚本所在目录: {current_dir}")
    
    # 轨迹文件路径 - 基于脚本目录构建
    localization_tum = os.path.join(current_dir, 'localization.tum')
    odometry_tum = os.path.join(current_dir, 'odometry.tum')
    
    # 输出图像文件 - 基于脚本目录构建（修改为分别保存里程计和定位图像）
    odometry_image = os.path.join(current_dir, 'odometry.png')
    localization_image = os.path.join(current_dir, 'localization.png')

    try:
        # 检查文件存在性
        if not os.path.exists(localization_tum):
            raise FileNotFoundError(f"定位轨迹文件 {localization_tum} 不存在")
        if not os.path.exists(odometry_tum):
            raise FileNotFoundError(f"里程计轨迹文件 {odometry_tum} 不存在")
            
        # 读取定位轨迹数据
        localization_data = np.loadtxt(localization_tum)
        local_x = localization_data[:, 1]  # 第2列是X坐标
        local_y = localization_data[:, 2]  # 第3列是Y坐标
        local_z = localization_data[:, 3]  # 第4列是Z坐标
        
        # 读取里程计轨迹数据
        odometry_data = np.loadtxt(odometry_tum)
        odom_x = odometry_data[:, 1]  # 第2列是X坐标
        odom_y = odometry_data[:, 2]  # 第3列是Y坐标
        odom_z = odometry_data[:, 3]  # 第4列是Z坐标
        
        print(f"成功读取定位轨迹数据点: {len(localization_data)} 个")
        print(f"成功读取里程计轨迹数据点: {len(odometry_data)} 个")

        # 绘制里程计轨迹
        fig1 = plot_trajectory(
            odom_x, odom_y, odom_z, 
            title_prefix='Odometry', 
            color='r', 
            output_file=odometry_image
        )
        
        # 绘制定位轨迹
        fig2 = plot_trajectory(
            local_x, local_y, local_z, 
            title_prefix='Localization', 
            color='b', 
            output_file=localization_image
        )
        
        print("所有图像已成功保存，无窗口显示")
        
    except FileNotFoundError as e:
        print(f"错误: {str(e)}")
    except Exception as e:
        print(f"处理数据时发生错误: {str(e)}") 