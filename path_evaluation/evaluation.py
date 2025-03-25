import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 读取CSV文件
odometry_data = pd.read_csv('src/fast_lio_localization/path_evaluation/odometry_trajectory.csv')
localization_data = pd.read_csv('src/fast_lio_localization/path_evaluation/localization_trajectory.csv')

# 计算误差
error_x = localization_data['x'] - odometry_data['x']
error_y = localization_data['y'] - odometry_data['y']
error = np.sqrt(error_x**2 + error_y**2)

# 可视化
plt.figure(figsize=(12, 6))

# 绘制里程计轨迹
plt.plot(odometry_data['x'], odometry_data['y'], label='Odometry', color='blue')

# 绘制定位轨迹
plt.plot(localization_data['x'], localization_data['y'], label='Localization', color='red')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Odometry vs Localization Trajectories')
plt.legend()
plt.grid(True)
plt.axis('equal')

# 保存图片
plt.savefig('src/fast_lio_localization/path_evaluation/trajectory_comparison.png')
print("轨迹比较图已保存至 src/fast_lio_localization/doc/trajectory_comparison.png")

# 打印完成信息
print("轨迹比较图绘制完成")