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

# 获取当前脚本目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 设置文件路径 - 使用绝对路径确保文件能够被找到
ref_tum = os.path.join(current_dir, "odometry.tum")  # 参考轨迹文件路径
est_tum = os.path.join(current_dir, "localization.tum")  # 估计轨迹文件路径
output_dir = current_dir  # 输出目录，保存在当前文件夹下

print(f"参考轨迹文件路径: {ref_tum}")
print(f"估计轨迹文件路径: {est_tum}")
print(f"输出目录: {output_dir}")

def create_stats_pdf(ape_stats, rpe_stats, output_file):
    """创建包含APE和RPE统计数据的PDF"""
    # 设置matplotlib不显示图形界面
    plt.switch_backend('Agg')
    
    plt.figure(figsize=(10, 6))
    plt.axis('off')
    
    # 创建表格数据
    table_data = []
    headers = ["Metric", "APE (m)", "RPE (m)"]
    
    # 定义要显示的指标及其显示名称
    metrics = ["rmse", "mean", "median", "std", "min", "max"]
    metric_names = ["RMSE", "Mean", "Median", "Std Dev", "Min", "Max"]
    
    # 遍历每个指标，获取APE和RPE的值
    for metric, name in zip(metrics, metric_names):
        # 从统计数据中获取值，如果不存在则显示"N/A"
        ape_value = ape_stats.get(metric, "N/A")
        rpe_value = rpe_stats.get(metric, "N/A")
        
        # 如果值是浮点数，则格式化为4位小数
        if isinstance(ape_value, float):
            ape_value = f"{ape_value:.4f}"
        if isinstance(rpe_value, float):
            rpe_value = f"{rpe_value:.4f}"
            
        # 将指标名称和对应的APE、RPE值添加到表格数据中
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

def extract_stats_from_file(file_path):
    """从统计结果文件中提取统计数据"""
    stats = {}
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            if "stats" in data:
                stats = data["stats"]
    except Exception as e:
        print(f"读取统计文件时出错: {e}")
    return stats

def visualize_trajectories(ref_tum, est_tum, output_dir="./results"):
    """使用evo可视化轨迹并生成综合报告"""
    try:
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 创建临时目录存放中间文件
        temp_dir = tempfile.mkdtemp()
        
        # 首先设置evo配置，使用白底黑线的可视化样式
        subprocess.run(["evo_config", "set", "plot_seaborn_style", "ticks"], check=True)
        
        # 1. 使用evo_traj生成轨迹对比图
        print("生成轨迹对比图...")
        traj_cmd = [
            "evo_traj", "tum", ref_tum, est_tum,
            "--ref", ref_tum,
            "--align",
            "--plot",
            "--plot_mode", "xyz",
            "--save_plot", f"{temp_dir}/trajectory_comparison.pdf",
            "--save_as_tum"
        ]
        
        subprocess.run(traj_cmd, check=True)
        print("轨迹对比图已生成")
        
        # 2. 计算APE并保存结果
        print("计算APE指标...")
        ape_cmd = [
            "evo_ape", "tum", ref_tum, est_tum,
            "--align",
            "--plot",
            "--plot_mode", "xyz",
            "--save_plot", f"{temp_dir}/ape_results.pdf",
            "--save_results", f"{temp_dir}/ape_results.zip",
            "--no_warnings"
        ]
        
        subprocess.run(ape_cmd, check=True)
        print("APE计算已完成")
        
        # 3. 计算RPE并保存结果
        print("计算RPE指标...")
        rpe_cmd = [
            "evo_rpe", "tum", ref_tum, est_tum,
            "--align",
            "--plot",
            "--plot_mode", "xyz",
            "--save_plot", f"{temp_dir}/rpe_results.pdf",
            "--save_results", f"{temp_dir}/rpe_results.zip",
            "--no_warnings"
        ]
        
        subprocess.run(rpe_cmd, check=True)
        print("RPE计算已完成")
        
        # 4. 解压结果文件，获取统计数据
        import zipfile
        with zipfile.ZipFile(f"{temp_dir}/ape_results.zip", 'r') as zip_ref:
            zip_ref.extractall(f"{temp_dir}/ape_results")
        
        with zipfile.ZipFile(f"{temp_dir}/rpe_results.zip", 'r') as zip_ref:
            zip_ref.extractall(f"{temp_dir}/rpe_results")
        
        # 5. 提取APE和RPE的统计数据
        ape_stats = extract_stats_from_file(f"{temp_dir}/ape_results/stats.json")
        rpe_stats = extract_stats_from_file(f"{temp_dir}/rpe_results/stats.json")
        
        # 6. 创建统计数据表格
        create_stats_pdf(ape_stats, rpe_stats, f"{temp_dir}/stats_table.pdf")
        
        # 7. 合并所有PDF
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
        import traceback
        traceback.print_exc()
        return False

# 检查文件是否存在
if not os.path.exists(ref_tum):
    print(f"错误：参考轨迹文件 {ref_tum} 不存在")
else:
    if not os.path.exists(est_tum):
        print(f"错误：估计轨迹文件 {est_tum} 不存在")
    else:
        # 可视化轨迹
        visualize_trajectories(ref_tum, est_tum, output_dir)