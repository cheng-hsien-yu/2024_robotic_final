import pandas as pd
import numpy as np

# 讀取CSV文件
file_path = '2024_robotics/scripts/data.csv'  # 修改成你的文件路徑
data = pd.read_csv(file_path)

# 提取UKF和Local的位置數據（修改列名）
ukf_position = data[['ukf_pos_x', 'ukf_pos_y', 'ukf_pos_z']].values  # 修改成實際的UKF列名
local_position = data[['local_pos_x', 'local_pos_y', 'local_pos_z']].values  # 修改成實際的Local列名

# 計算兩者之間的誤差
error = ukf_position - local_position

# 計算均方誤差（Mean Squared Error, MSE）
mse = np.mean(error ** 2, axis=0)

print("UKF和Local位置的均方誤差 (MSE):")
print(f"X軸 MSE: {mse[0]:.2f}")
print(f"Y軸 MSE: {mse[1]:.2f}")
