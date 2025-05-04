#!/usr/bin/env python3
"""
轨迹可视化脚本 - 使用evo工具包比较两条TUM格式轨迹并生成综合报告
"""

import os
import subprocess
import tempfile
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import argparse  # 添加argparse模块

# 设置默认matplotlib参数
plt.rcParams['grid.alpha'] = 0.2  # 网格线透明度
plt.rcParams['grid.linewidth'] = 0.5  # 更细的网格线
plt.rcParams['xtick.major.size'] = 4.0  # 设置X轴主要刻度大小
plt.rcParams['ytick.major.size'] = 4.0  # 设置Y轴主要刻度大小
plt.rcParams['ytick.minor.visible'] = False  # 隐藏次要刻度
plt.rcParams['xtick.minor.visible'] = False  # 隐藏次要刻度
# 设置所有文字大小为26
plt.rcParams['font.size'] = 26
plt.rcParams['axes.titlesize'] = 26
plt.rcParams['axes.labelsize'] = 26
plt.rcParams['xtick.labelsize'] = 26
plt.rcParams['ytick.labelsize'] = 26
plt.rcParams['legend.fontsize'] = 26
# 设置坐标轴标签的间距
plt.rcParams['axes.labelpad'] = 20  # 增加标签的间距
# 设置图像DPI和输出质量
plt.rcParams['figure.dpi'] = 300  # 设置显示DPI
plt.rcParams['savefig.dpi'] = 300  # 设置保存DPI
plt.rcParams['savefig.bbox'] = 'tight'  # 紧凑布局
plt.rcParams['savefig.pad_inches'] = 0.1  # 边缘填充
# 设置不显示图形界面 - 将在参数解析后决定是否启用

# 获取当前脚本目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 设置文件路径 - 使用绝对路径确保文件能够被找到
ref_tum = os.path.join(current_dir, "odometry.tum")  # 参考轨迹文件路径
est_tum = os.path.join(current_dir, "localization.tum")  # 估计轨迹文件路径
output_dir = os.path.join(current_dir, "evo_results")  # 输出目录，设置为evo_results子文件夹

def preprocess_trajectory_files(input_file, output_file):
    """预处理轨迹文件，使时间戳从0开始"""
    data = np.loadtxt(input_file)
    if len(data) > 0:
        first_timestamp = data[0, 0]  # 获取第一个时间戳
        data[:, 0] = data[:, 0] - first_timestamp  # 所有时间戳减去第一个时间戳
        np.savetxt(output_file, data, fmt='%.6f')
        print(f"已处理轨迹文件，时间从0开始: {output_file}")
    return output_file

def visualize_trajectories(ref_tum, est_tum, output_dir="./evo_results", show_plot=False):
    """使用evo可视化轨迹并生成综合报告
    
    Args:
        ref_tum: 参考轨迹文件路径
        est_tum: 估计轨迹文件路径
        output_dir: 输出目录路径
        show_plot: 是否显示绘图窗口
    """
    try:
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 创建临时目录存放中间文件
        temp_dir = tempfile.mkdtemp()
        
        # 创建自定义matplotlibrc文件，让轨迹图中显示更稀疏的网格
        mplrc_path = os.path.join(temp_dir, "matplotlibrc")
        with open(mplrc_path, "w") as f:
            f.write("ytick.direction : in\n")  # 刻度向内
            f.write("xtick.direction : in\n")  # 刻度向内
            f.write("grid.alpha : 0.2\n")  # 非常透明的网格线
            f.write("grid.linewidth : 0.5\n")  # 更细的网格线
            f.write("xtick.minor.visible : False\n")  # 不显示X次要刻度
            f.write("ytick.minor.visible : False\n")  # 不显示Y次要刻度
            f.write("ztick.minor.visible : False\n")  # 不显示Z次要刻度
            f.write("axes.grid : True\n")
            f.write("axes.grid.which : major\n")  # 只显示主要网格线
            f.write("font.size : 26\n")  # 设置默认字体大小为26
            f.write("axes.titlesize : 26\n")  # 设置标题字体大小为26
            f.write("axes.labelsize : 26\n")  # 设置轴标签字体大小为26
            f.write("xtick.labelsize : 26\n")  # 设置x轴刻度标签字体大小为26
            f.write("ytick.labelsize : 26\n")  # 设置y轴刻度标签字体大小为26
            f.write("legend.fontsize : 26\n")  # 设置图例字体大小为26
            f.write("axes.labelpad : 20\n")  # 增加坐标轴标签的间距
            f.write("figure.dpi : 300\n")  # 设置显示DPI
            f.write("savefig.dpi : 300\n")  # 设置保存DPI
            f.write("savefig.bbox : tight\n")  # 紧凑布局
            f.write("savefig.pad_inches : 0.1\n")  # 边缘填充
            f.write("axes3d.xaxis.panecolor : (0.95, 0.95, 0.95, 0.1)\n")  # 设置3D坐标轴面板颜色和透明度
            f.write("axes3d.yaxis.panecolor : (0.95, 0.95, 0.95, 0.1)\n")
            f.write("axes3d.zaxis.panecolor : (0.95, 0.95, 0.95, 0.1)\n")
            # 根据参数决定是否使用非交互式后端
            if not show_plot:
                f.write("backend : Agg\n")  # 只在配置文件中设置非交互式后端
            
        # 保存原始环境变量，以便后续恢复
        original_matplotlibrc = os.environ.get("MATPLOTLIBRC", "")
        original_mplbackend = os.environ.get("MPLBACKEND", "")
        
        # 在子进程中使用自定义配置，不影响全局环境
        env = os.environ.copy()
        env["MATPLOTLIBRC"] = temp_dir
        if not show_plot:
            env["MPLBACKEND"] = "Agg"

        # 设置evo配置 - 使用修改后的环境变量执行命令

        # 背景风格设置
        subprocess.run(["evo_config", "set", "plot_seaborn_style", "ticks"], check=True, env=env)
        
        if not show_plot:
            subprocess.run(["evo_config", "set", "plot_backend", "Agg"], check=True, env=env)
        
        # 设置图片字体大小
        # 字体大小：
        # 
    
        # 图形尺寸
        subprocess.run(["evo_config", "set", "plot_figsize", "12", "8"], check=True, env=env)  
        
        # 字体比例设置值：
        # 轨迹对比图：1
        # xyz和rpy图：2
        # 速度图：1
        # ape和rpe：1.7
        subprocess.run(["evo_config", "set", "plot_fontscale", "1.7"], check=True, env=env)
        
        # 配置evo工具包中轨迹线的宽度
        subprocess.run(["evo_config", "set", "plot_linewidth", "1.0"], check=True, env=env)
        
        # 构建基本命令参数
        plot_args = ["--plot", "--plot_mode", "xyz"]
        
        # 只在evo_traj命令中使用--show参数
        traj_plot_args = plot_args.copy()
        if show_plot:
            traj_plot_args.append("--show")
        
        # 在visualize_trajectories函数中，创建处理后的文件
        temp_ref_tum = os.path.join(output_dir, "Odometry.tum")
        temp_est_tum = os.path.join(output_dir, "Localization.tum")
        preprocess_trajectory_files(ref_tum, temp_ref_tum)
        preprocess_trajectory_files(est_tum, temp_est_tum)

        # 1. 使用evo_traj生成轨迹对比图 - 使用修改后的环境变量
        
        print("生成轨迹对比图...") # 打印提示信息，表示开始生成轨迹对比图
        traj_png = os.path.join(output_dir, "trajectory_comparison.png") # 定义轨迹对比图的保存路径
        traj_cmd = [
            "evo_traj", "tum", temp_ref_tum, temp_est_tum,
            "--ref", temp_ref_tum,
            "--align",
        ] + traj_plot_args + ["--save_plot", traj_png, "--no_warnings"]
        
        subprocess.run(traj_cmd, check=True, env=env) # 执行evo_traj命令，使用修改后的环境变量
        print(f"轨迹对比图已保存至: {traj_png}") # 打印保存成功的提示信息和文件路径
        
        # 2. 计算APE并保存结果 - 使用修改后的环境变量
        print("计算APE指标...")
        ape_png = os.path.join(output_dir, "ape_results.png")
        ape_cmd = [
            "evo_ape", "tum", temp_ref_tum, temp_est_tum,
            "--align",
        ] + plot_args + ["--save_plot", ape_png, "--no_warnings"]
        
        subprocess.run(ape_cmd, check=True, env=env)
        print(f"APE结果图已保存至: {ape_png}")
        
        # 3. 计算RPE并保存结果 - 使用修改后的环境变量
        print("计算RPE指标...")
        rpe_png = os.path.join(output_dir, "rpe_results.png")
        rpe_cmd = [
            "evo_rpe", "tum", temp_ref_tum, temp_est_tum,
            "--align",
        ] + plot_args + ["--save_plot", rpe_png, "--no_warnings"]
        
        subprocess.run(rpe_cmd, check=True, env=env)
        print(f"RPE结果图已保存至: {rpe_png}")
        
        print("\n评估结果总结:")
        print(f"1. 轨迹对比图: {traj_png}")
        print(f"2. APE结果图: {ape_png}")
        print(f"3. RPE结果图: {rpe_png}")
        
        # 在所有处理完成后重置evo配置
        try:
            print("重置evo配置...")
            if os.name == 'nt':  # Windows系统
                reset_cmd = "echo y | evo_config reset"
            else:  # Unix/Linux/Mac系统
                reset_cmd = "yes y | evo_config reset"
            subprocess.run(reset_cmd, shell=True, check=True, env=env)
        except Exception as e:
            print(f"重置evo配置时出错: {e}")
        
        return True
    except subprocess.CalledProcessError as e:
        print(f"执行evo命令时出错: {e}")
        if e.stderr:
            print(f"错误详情: {e.stderr}")
        
        # 即使出错也尝试重置配置
        try:
            print("重置evo配置...")
            if os.name == 'nt':  # Windows系统
                reset_cmd = "echo y | evo_config reset"
            else:  # Unix/Linux/Mac系统
                reset_cmd = "yes y | evo_config reset"
            subprocess.run(reset_cmd, shell=True, check=True, env=env)
        except Exception as reset_error:
            print(f"重置evo配置时出错: {reset_error}")
        
        return False
    except Exception as e:
        print(f"可视化轨迹时出错: {e}")
        import traceback
        traceback.print_exc()
        
        # 即使出错也尝试重置配置
        try:
            print("重置evo配置...")
            if os.name == 'nt':  # Windows系统
                reset_cmd = "echo y | evo_config reset"
            else:  # Unix/Linux/Mac系统
                reset_cmd = "yes y | evo_config reset"
            subprocess.run(reset_cmd, shell=True, check=True, env=env)
        except Exception as reset_error:
            print(f"重置evo配置时出错: {reset_error}")
        
        return False

def main():
    """主函数，处理命令行参数并执行轨迹可视化"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="轨迹可视化与评估工具")
    parser.add_argument("--show", action="store_true", help="显示绘图窗口（默认不显示）")
    parser.add_argument("--ref", default=ref_tum, help=f"参考轨迹文件路径（默认: {ref_tum}）")
    parser.add_argument("--est", default=est_tum, help=f"估计轨迹文件路径（默认: {est_tum}）")
    parser.add_argument("--output", default=output_dir, help=f"输出目录路径（默认: {output_dir}）")
    
    args = parser.parse_args()
    
    # 打印配置信息
    print(f"参考轨迹文件路径: {args.ref}")
    print(f"估计轨迹文件路径: {args.est}")
    print(f"输出目录: {args.output}")
    print(f"是否显示绘图窗口: {args.show}")
    
    # 检查文件是否存在
    if not os.path.exists(args.ref):
        print(f"错误：参考轨迹文件 {args.ref} 不存在")
    else:
        if not os.path.exists(args.est):
            print(f"错误：估计轨迹文件 {args.est} 不存在")
        else:
            # 可视化轨迹
            visualize_trajectories(args.ref, args.est, args.output, args.show)

    # 最后尝试重置配置
    try:
        print("\n重置全局evo配置...")
        # 使用python的subprocess模块，通过communicate方法提供输入
        reset_process = subprocess.Popen(["evo_config", "reset"], 
                                        stdin=subprocess.PIPE, 
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE,
                                        text=True)
        stdout, stderr = reset_process.communicate(input="y\n")
        print(stdout)
        if reset_process.returncode == 0:
            print("evo配置已成功重置")
        else:
            print(f"重置失败: {stderr}")
    except Exception as e:
        print(f"重置evo配置失败: {e}")

if __name__ == "__main__":
    main()