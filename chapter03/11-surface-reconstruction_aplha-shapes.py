"""
http://www.open3d.org/docs/release/tutorial/Advanced/surface_reconstruction.html

"""

import open3d as o3d
import numpy as np
from common import open3d_tutorial as o3dtut

mesh = o3dtut.get_bunny_mesh()
"""
 Function to sample points from the mesh, where each point has approximately the same distance 
 to the neighbouring points (blue noise). Method is based on Yuksel, 
 "Sample Elimination for Generating Poisson Disk Sample Sets", EUROGRAPHICS, 2015.
用于从网格采样点的功能，其中每个点到相邻点的距离大约相同（蓝噪声）。
方法基于Yuksel，“消除泊松圆盘样本集的样本消除”，EUROGRAPHICS，2015年。
"""
pcd = mesh.sample_points_poisson_disk(750)
o3d.visualization.draw_geometries([pcd])
alpha = 0.03
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha, tetra_mesh, pt_map)

    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)