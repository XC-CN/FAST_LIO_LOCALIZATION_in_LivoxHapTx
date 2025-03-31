#!/usr/bin/env python3
"""
轨迹可视化脚本 - 使用evo工具包比较两条TUM格式轨迹并生成综合报告
"""

import os
import subprocess
import tempfile
from PyPDF2 import PdfMerger
import matplotlib.pyplot as plt
import numpy as np
import json

# 设置文件路径
ref_tum = "src/fast_lio_localization/path_evaluation/odometry.tum"  # 替换为您的参考轨迹文件路径
est_tum = "src/fast_lio_localization/path_evaluation/localization.tum"  # 替换为您的估计轨迹文件路径
output_dir = "src/fast_lio_localization/path_evaluation"  # 输出目录，保存在path_evaluation文件夹下

def visualize_trajectories(ref_tum, est_tum, output_dir="./results"):
    """使用evo可视化轨迹并生成综合报告"""
    try:
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 创建临时目录存放中间文件
        temp_dir = tempfile.mkdtemp()
        
        # 构建evo命令 - 轨迹可视化
        traj_cmd = [
            "evo_traj", "tum", 
            ref_tum, est_tum, 
            "--ref", ref_tum, 
            "--align", 
            "--plot", 
            "--plot_mode=xyz", 
            "--save_plot", f"{temp_dir}/trajectory_comparison.pdf"
        ]
        
        # 计算APE并输出详细统计信息
        ape_cmd = [
            "evo_ape", "tum", 
            ref_tum, est_tum, 
            "-a", 
            "--plot", 
            "--plot_mode=xyz",
            "--save_plot", f"{temp_dir}/ape_results.pdf",
            "--no_warnings",
            "--verbose"  # 输出详细统计信息
        ]

        # 计算RPE
        rpe_cmd = [
            "evo_rpe", "tum", 
            ref_tum, est_tum, 
            "-a", 
            "--plot", 
            "--plot_mode=xyz",
            "--save_plot", f"{temp_dir}/rpe_results.pdf",
            "--no_warnings",
            "--verbose"
        ]
        
        # 执行命令
        print("执行命令:", " ".join(traj_cmd))
        traj_result = subprocess.run(traj_cmd, check=True, capture_output=True, text=True)
        print("轨迹可视化成功完成")
        
        print("执行命令:", " ".join(ape_cmd))
        ape_result = subprocess.run(ape_cmd, check=True, capture_output=True, text=True)
        print("APE计算成功完成")
        ape_output = ape_result.stdout
        
        print("执行命令:", " ".join(rpe_cmd))
        rpe_result = subprocess.run(rpe_cmd, check=True, capture_output=True, text=True)
        print("RPE计算成功完成")
        rpe_output = rpe_result.stdout
        
        # 提取APE和RPE的统计数据
        ape_stats = extract_stats(ape_output)
        rpe_stats = extract_stats(rpe_output)
        
        # 创建统计数据表格
        create_stats_pdf(ape_stats, rpe_stats, f"{temp_dir}/stats_table.pdf")
        
        # 合并所有PDF
        merger = PdfMerger()
        
        # 添加统计表格
        merger.append(f"{temp_dir}/stats_table.pdf")
        
        # 添加轨迹对比图
        merger.append(f"{temp_dir}/trajectory_comparison.pdf")
        
        # 添加APE结果
        merger.append(f"{temp_dir}/ape_results.pdf")
        
        # 添加RPE结果
        merger.append(f"{temp_dir}/rpe_results.pdf")
        
        # 保存合并后的PDF
        merger.write(f"{output_dir}/trajectory_evaluation_report.pdf")
        merger.close()
        
        print(f"综合评估报告已保存至: {output_dir}/trajectory_evaluation_report.pdf")
        
        return True
    except subprocess.CalledProcessError as e:
        print(f"执行evo命令时出错: {e}")
        if e.stderr:
            print(f"错误详情: {e.stderr}")
        return False
    except Exception as e:
        print(f"可视化轨迹时出错: {e}")
        return False

def extract_stats(output_text):
    """从evo输出中提取统计数据"""
    stats = {}
    
    # 查找统计数据部分
    if "stats" in output_text:
        stats_section = output_text.split("stats")[1].split("\n")[1:7]
        for line in stats_section:
            if ":" in line:
                key, value = line.split(":", 1)
                stats[key.strip()] = float(value.strip())
    
    return stats

def create_stats_pdf(ape_stats, rpe_stats, output_file):
    """创建包含APE和RPE统计数据的PDF"""
    plt.figure(figsize=(10, 6))
    plt.axis('off')
    
    # 创建表格数据
    table_data = []
    headers = ["Metric", "APE (m)", "RPE (m)"]
    
    metrics = ["rmse", "mean", "median", "std", "min", "max"]
    metric_names = ["RMSE", "Mean", "Median", "Std Dev", "Min", "Max"]
    
    for metric, name in zip(metrics, metric_names):
        ape_value = ape_stats.get(metric, "N/A")
        rpe_value = rpe_stats.get(metric, "N/A")
        
        if isinstance(ape_value, float):
            ape_value = f"{ape_value:.4f}"
        if isinstance(rpe_value, float):
            rpe_value = f"{rpe_value:.4f}"
            
        table_data.append([name, ape_value, rpe_value])
    
    # 创建表格
    table = plt.table(
        cellText=table_data,
        colLabels=headers,
        loc='center',
        cellLoc='center',
        colWidths=[0.3, 0.3, 0.3]
    )
    
    # 设置表格样式
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 2)
    
    # 添加标题
    plt.title("Trajectory Evaluation Statistics", fontsize=16, pad=20)
    
    # 保存为PDF
    plt.savefig(output_file, bbox_inches='tight')
    plt.close()



# 检查文件是否存在
if not os.path.exists(ref_tum):
    print(f"错误：参考轨迹文件 {ref_tum} 不存在")
else:
    if not os.path.exists(est_tum):
        print(f"错误：估计轨迹文件 {est_tum} 不存在")
    else:
        # 可视化轨迹
        visualize_trajectories(ref_tum, est_tum, output_dir)