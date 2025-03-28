#!/usr/bin/env python3
"""
轨迹可视化脚本 - 使用evo工具包比较两条TUM格式轨迹
"""

import os
import subprocess

def visualize_trajectories(ref_tum, est_tum, output_dir="./results"):
    """使用evo可视化轨迹"""
    try:
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 构建evo命令
        cmd = [
            "evo_traj", "tum", 
            ref_tum, est_tum, 
            "--ref", ref_tum, 
            "--align", 
            "--plot", 
            "--plot_mode=xyz", 
            "--save_plot", f"{output_dir}/trajectory_comparison.pdf"
        ]
        
        # 执行命令
        print("执行命令:", " ".join(cmd))
        result = subprocess.run(cmd, check=True)
        print("轨迹可视化成功完成")
        
        return True
    except subprocess.CalledProcessError as e:
        print(f"执行evo命令时出错: {e}")
        return False
    except Exception as e:
        print(f"可视化轨迹时出错: {e}")
        return False

# 设置文件路径
ref_tum = "src/fast_lio_localization/path_evaluation/odometry.tum"  # 替换为您的参考轨迹文件路径
est_tum = "src/fast_lio_localization/path_evaluation/localization.tum"  # 替换为您的估计轨迹文件路径
output_dir = "src/fast_lio_localization/path_evaluation/results"  # 输出目录，保存在path_evaluation文件夹下

# 检查文件是否存在
if not os.path.exists(ref_tum):
    print(f"错误：参考轨迹文件 {ref_tum} 不存在")
else:
    if not os.path.exists(est_tum):
        print(f"错误：估计轨迹文件 {est_tum} 不存在")
    else:
        # 可视化轨迹
        visualize_trajectories(ref_tum, est_tum, output_dir)