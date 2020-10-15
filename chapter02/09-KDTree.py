"""
下面的代码读取一个点云并构建一个KDTree。 这是用于以下最近邻居查询的预处理步骤。

"""

import open3d as o3d
import numpy as np


print("Testing kdtree in open3d ...")
print("Load a point cloud and paint it gray.")
pcd = o3d.io.read_point_cloud("../TestData/Feature/cloud_bin_0.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

# Find neighboring points 我们选择第1500个点作为锚点并将其涂成红色。

print("Paint the 1500th point red.")
pcd.colors[1500] = [1, 0, 0]

"""
Using search_knn_vector_3d

函数search_knn_vector_3d返回锚点的k个最近邻居的索引列表。 这些相邻点涂有蓝色。 
请注意，我们将pcd.colors转换为numpy数组以批量访问点颜色，并向所有选定点广播蓝色[0，0，1]。 我们跳过第一个索引，因为它是锚点本身。
"""
print("Find its 200 nearest neighbors, paint blue.")
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

"""
Using search_radius_vector_3d

同样，我们可以使用search_radius_vector_3d查询所有距锚点距离小于给定半径的点。 我们将这些点涂成绿色。
"""
print("Find its neighbors with distance less than 0.2, paint green.")
[k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

print("Visualize the point cloud.")
o3d.visualization.draw_geometries([pcd])

"""
除了KNN搜索search_knn_vector_3d和RNN搜索search_radius_vector_3d外，
Open3D还提供了混合搜索功能search_hybrid_vector_3d。 它最多返回k个与锚点的距离小于给定半径的最近邻居。 
该功能结合了KNN搜索和RNN搜索的条件。 在某些文献中将其称为RKNN搜索。 在许多实际情况下，它具有性能优势，并且在许多Open3D功能中大量使用。
"""