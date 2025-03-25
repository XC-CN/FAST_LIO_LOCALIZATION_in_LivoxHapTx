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

if __name__ == '__main__':
    bag_file = 'localization_test.bag'  # 替换为您的 rosbag 文件名
    csv_file = 'src/fast_lio_localization/path_evaluation/odometry_trajectory.csv'  # 输出 CSV 文件名

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
            
            # 提取数据
            trajectory_data = []
            for topic, msg, t in bag.read_messages(topics=matched_topics):
                # 添加类型检查
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y
                    z = msg.pose.pose.position.z
                    trajectory_data.append([t.to_sec(), x, y, z])
            
            print(f"成功提取 {len(trajectory_data)} 条轨迹数据")

            # 保存CSV
            with open(csv_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'z'])
                writer.writerows(trajectory_data)
                print(f"数据已保存至 {csv_file}")

            # ------------------ 可视化部分 ------------------
            # 从CSV文件加载数据，跳过第一行（标题行）
            data = np.loadtxt(csv_file, delimiter=',', skiprows=1)
            # 提取X坐标（第2列）
            x = data[:, 1]
            # 提取Y坐标（第3列）
            y = data[:, 2]

            plt.figure(figsize=(10, 6))
            plt.plot(x, y, marker='o', linestyle='-', markersize=2)
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Odometry Trajectory')
            plt.grid(True)
            plt.axis('equal')
            
            # 保存图片
            image_file = 'src/fast_lio_localization/doc/odometry_trajectory.png'
            plt.savefig(image_file)
            print(f"轨迹图片已保存至 {image_file}")
            
         
            

    except FileNotFoundError as e:
        print(f"错误: {str(e)}")
    except Exception as e:
        print(f"处理数据时发生错误: {str(e)}")
