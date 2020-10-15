"""
http://www.open3d.org/docs/release/tutorial/Advanced/surface_reconstruction.html#Ball-pivoting
一种与Alpha形状相关的表面重建方法是球旋转算法（ ball pivoting algorithm  BPA）[Bernardini1999]。
直观地，考虑一个我们将其放到点云上的具有给定半径的3D球。
如果命中了任何3个点（并且没有跌落到这3个点），它将创建一个三角形。
然后，该算法从现有三角形的边缘开始旋转，每当它击中3个不落球的点时，我们都会创建另一个三角形。

This algorithm assumes that the PointCloud has normals.
"""

import open3d as o3d
import numpy as np
from common import open3d_tutorial as o3dtut

gt_mesh = o3dtut.get_bunny_mesh()
gt_mesh.compute_vertex_normals()
pcd = gt_mesh.sample_points_poisson_disk(3000)
o3d.visualization.draw_geometries([pcd], point_show_normal=True)

radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])