import open3d as o3d
import numpy as np
from open3d.open3d_pybind.geometry import PointCloud
from open3d.open3d_pybind.utility import Vector3dVector

print("Load a ply point cloud, print it, and render it")
# pcd = o3d.io.read_point_cloud("../TestData/fragment.ply")
pcd = o3d.io.read_point_cloud("../TestData/ss/s2.pcd")
print(pcd)
print(type(pcd))
print(type(pcd.points))

asarray = np.asarray(pcd.points)
asarray /= 1000.0
pcd.points = Vector3dVector(asarray)

# print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

# 降采样 ------------------------------------------------ Voxel downsampling
downsample_pcd = pcd.voxel_down_sample(voxel_size=0.05)
o3d.visualization.draw_geometries([downsample_pcd])

if not isinstance(downsample_pcd, PointCloud): exit(-1)

# 计算法向量 ------------------------------------------------ Access estimated vertex normal
downsample_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
downsample_pcd.orient_normals_towards_camera_location()
o3d.visualization.draw_geometries([downsample_pcd], point_show_normal=True)