import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import cKDTree
import time

# 记录程序开始时间
start_time = time.time()

# 加载点云
print("->开始加载点云...")
reality = o3d.io.read_point_cloud("fragment.ply")
background = o3d.io.read_point_cloud("background.ply")

print(reality.points)

# 体素下采样以减少点云大小
print("->正在对点云进行下采样...")
voxel_size = 0.05
reality = reality.voxel_down_sample(voxel_size)
background = background.voxel_down_sample(voxel_size)

# 获取下采样后的点云数据
points1 = np.asarray(reality.points)
points2 = np.asarray(background.points)
colors1 = np.asarray(reality.colors)
colors2 = np.asarray(background.colors)

# 创建KDTree以加速查找
print("->正在进行场地滤波...")
kdtree2 = cKDTree(points2)

# 批量查询，找到点云1中每个点在点云2中的最近点
distances, indices = kdtree2.query(points1, k=1)

# 根据距离阈值过滤点
threshold = 0.001
mask = distances >= threshold  # 保留距离大于阈值的点

# 使用掩码过滤点和颜色
points1_filtered = points1[mask]
colors1_filtered = colors1[mask]

# 更新点云1
reality.points = o3d.utility.Vector3dVector(points1_filtered)
reality.colors = o3d.utility.Vector3dVector(colors1_filtered)  # 保留颜色信息

# 记录滤波过程的时间
filter_time = time.time()
print(f"场地滤波花费的时间：{filter_time - start_time:.4f}秒")


# 创建一个小球体来表示点云的中心
# sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)  # 半径可以调整
# sphere.translate(center)  # 将球体平移到点云的中心位置
# sphere.paint_uniform_color([1, 0, 0])  # 设置颜色为红色
# 可视化点云和红色中心
# o3d.visualization.draw_geometries([reality, sphere])

dbscan_time = time.time()
print("->正在DBSCAN聚类...")
eps = 0.08           # 同一聚类中最大点间距
min_points = 9     # 有效聚类的最小点数
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(reality.cluster_dbscan(eps, min_points, print_progress=True))
max_label = labels.max()    # 获取聚类标签的最大值 [-1,0,1,2,...,max_label]，label = -1 为噪声，因此总聚类个数为 max_label + 1
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0  # labels = -1 的簇为噪声，以黑色显示
reality.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 记录总的运行时间
end_time = time.time()

print(f"聚类花费的时间为：{end_time - dbscan_time:.4f}秒")


print("->正在计算每个聚类的中心...")
cluster_centers = []
for label in range(max_label + 1):  # 不包括噪声点
    cluster_points = points1_filtered[labels == label]
    center = np.mean(cluster_points, axis=0)
    cluster_centers.append(center)
    print(f"类别{label+1}的中心坐标: {center}")

# 记录总的运行时间
end_time = time.time()
print(f"总的运行时间：{end_time - start_time:.4f}秒")

o3d.visualization.draw_geometries_with_vertex_selection([reality],
                                                        window_name="选点显示坐标",
                                                        width=1920,  # 窗口宽度
                                                        height=1080,  # 窗口高度
                                                        left=50,  # 窗口在屏幕上的水平位置
                                                        top=50)  # 设置缩放比例