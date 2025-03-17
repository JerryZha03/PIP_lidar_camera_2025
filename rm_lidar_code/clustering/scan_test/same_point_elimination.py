import open3d as o3d 
import numpy as np 
from scipy.spatial  import cKDTree 
from matplotlib import pyplot as plt
import time 


# 加载并预处理点云 
def load_and_preprocess(filename):
    pcd = o3d.io.read_point_cloud(filename) 
    pcd = pcd.voxel_down_sample(voxel_size=0.05) 
    pcd.paint_uniform_color([0.5,  0.5, 0.5])  # 统一灰色 
    return pcd 
 
print("->开始加载点云...")
# reality = load_and_preprocess("reality.ply") 
background = load_and_preprocess("background_league.ply") 

o3d.visualization.draw_geometries_with_vertex_selection([background])

# 场地滤波处理 
print("->场地滤波中...")
kdtree = cKDTree(np.asarray(background.points)) 
distances, _ = kdtree.query(np.asarray(reality.points),  k=1)
mask = distances >= 0.2  # 动态阈值 
reality = reality.select_by_index(np.where(mask)[0]) 

o3d.visualization.draw_geometries_with_vertex_selection([reality])

print("->半径滤波中...")
num_points = 50  # 邻域球内的最少点数，低于该值的点为噪声点
radius = 0.4    # 邻域半径大小

# 执行半径滤波，返回滤波后的点云sor_data和对应的索引ind
reality, ind = reality.remove_radius_outlier(num_points, radius)

# DBSCAN聚类优化 
print("->智能聚类分析中...")
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):  
    labels = np.array(reality.cluster_dbscan(eps=0.3,  min_points=100))
 
# ======== 新增功能：计算聚类中心 & 颜色渲染 ========
max_label = labels.max() 
valid_labels = [label for label in range(max_label+1) if np.sum(labels  == label) > 0]
 
# 生成颜色映射（使用tab20色系）
cmap = plt.cm.get_cmap("tab20") 
colors = np.zeros((len(labels),  3))
for i, label in enumerate(valid_labels):
    mask = labels == label 
    # 计算聚类中心 
    cluster_points = np.asarray(reality.points)[mask] 
    center = np.mean(cluster_points,  axis=0)
    print(f"类别 {i+1} 中心坐标：({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")
    # 分配颜色（跳过-1噪声点）
    colors[mask] = cmap(i % 20)[:3]  # 循环使用20种颜色 
 
# 噪声点保持灰色（RGB 0.5,0.5,0.5）
noise_mask = labels == -1 
colors[noise_mask] = [0.5, 0.5, 0.5]
 
# 更新点云颜色 
reality.colors  = o3d.utility.Vector3dVector(colors) 
 
# 可视化与输出 
print(f"发现有效聚类：{len(valid_labels)} 个")
o3d.visualization.draw_geometries([reality]) 