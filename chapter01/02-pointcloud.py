import open3d as o3d
import numpy as np
from open3d.open3d_pybind.geometry import PointCloud

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("../TestData/fragment.ply")
# pcd = o3d.io.read_point_cloud("../TestData/ss/s2.pcd")
print(pcd)
print(type(pcd))
print(type(pcd.points))

print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])
# 降采样 ------------------------------------------------ Voxel downsampling
downsample_pcd = pcd.voxel_down_sample(voxel_size=0.05)
# o3d.visualization.draw_geometries([downsample_pcd])

if not isinstance(downsample_pcd, PointCloud): exit(-1)

# 计算法向量 ------------------------------------------------ Access estimated vertex normal
# downsample_pcd.orient_normals_to_align_with_direction()
downsample_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downsample_pcd], point_show_normal=True)

# 加载多边形 ------------------------------------------------ Crop point cloud
volume = o3d.visualization.read_selection_polygon_volume("../TestData/Crop/cropped.json")
# 裁剪指定点云
chair = volume.crop_point_cloud(pcd)
# o3d.visualization.draw_geometries([chair])

# 点云涂色 ------------------------------------------------ Paint point cloud
if not isinstance(chair, PointCloud): exit(-1)
chair.paint_uniform_color([1, 0, 0.8]) #[R, G, B]  0.0 -> 1.0
# o3d.visualization.draw_geometries([chair])

# 绘制包容盒 ------------------------------------------------ Bounding Volumes
# Open3D实现了AxisAlignedBoundingBox(AABB) 和 OrientedBoundingBox(OBB)，它们也可以用于裁剪几何。
aabb = chair.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
obb = chair.get_oriented_bounding_box()
obb.color = (0, 1, 0)
# o3d.visualization.draw_geometries([chair, aabb, obb, downsample_pcd])






