import open3d as o3d
import numpy as np
from open3d.open3d_pybind.utility import Vector3dVector

print("Load a ply point cloud, print it, and render it")
# pcd = o3d.io.read_point_cloud("../TestData/ICP/cloud_bin_2.pcd")
pcd = o3d.io.read_point_cloud("../TestData/ss/s3.pcd")

pcd.points = Vector3dVector(np.asarray(pcd.points) / 1000.0)

o3d.visualization.draw_geometries([pcd])


print("Downsample the point cloud with a voxel of 0.02")
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.05)
o3d.visualization.draw_geometries([voxel_down_pcd])

print("Every 5th points are selected")
uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
o3d.visualization.draw_geometries([uni_down_pcd])


def display_inlier_outlier(cloud, ind):
    """
    The helper function uses select_down_sample that takes binary mask to output only the selected points.
    The selected points and the non-selected points are visualized.
    :param cloud: 输入点云
    :param ind:   被选中的点索引
    """
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

print("Statistical oulier removal")
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.0)
display_inlier_outlier(voxel_down_pcd, ind)

print("Radius oulier removal")
cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=30, radius=0.2)
display_inlier_outlier(voxel_down_pcd, ind)