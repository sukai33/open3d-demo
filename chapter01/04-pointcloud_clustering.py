# 官网链接： http://www.open3d.org/docs/release/tutorial/Basic/pointcloud.html#DBSCAN-clustering
# 算法论文： http://www.open3d.org/docs/release/tutorial/reference.html#Ester1996

"""
该算法在cluster_dbscan中实现，并且需要两个参数。
eps: 定义到群集中邻居的距离
min_points: 定义形成群集所需的最小点数。

该算法为所有点预先计算出eps-ε半径内的所有邻居。
如果选择epsilon太大，则可能需要大量内存。

该函数返回 labels标签列表，其中标签为-1的表示噪音。

"""

import open3d as o3d
import numpy as np
from open3d.open3d_pybind.geometry import PointCloud
import matplotlib.pyplot as plt

pcd:PointCloud = o3d.io.read_point_cloud("../TestData/fragment.ply")

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps = 0.02, min_points=10, print_progress=True))

max_label = labels.max()

print(f"point cloud 共有 {max_label + 1} 个簇")

colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

o3d.visualization.draw_geometries([pcd])


