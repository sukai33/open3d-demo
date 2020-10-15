"""
http://www.open3d.org/docs/release/tutorial/Advanced/surface_reconstruction.html#Poisson-surface-reconstruction

表面重建方法会产生不平滑的结果，因为PointCloud的点也是三角形网格的顶点，无需进行任何修改。
泊松曲面重构方法[Kazhdan2006]解决了正规化优化问题，以获得光滑表面。

Open3D实现了create_from_point_cloud_poisson方法，该方法基本上是Kazhdan代码的包装。
该函数的一个重要参数是深度，深度定义了用于表面重建的八叉树的深度，因此暗示了所得三角形网格的分辨率。 较高的深度值意味着具有更多细节的网格。

This algorithm assumes that the PointCloud has normals.
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from common import open3d_tutorial as o3dtut

pcd = o3dtut.get_eagle_pcd()
print(pcd)
o3d.visualization.draw_geometries([pcd])

print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
print(mesh)
o3d.visualization.draw_geometries([mesh])

print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
o3d.visualization.draw_geometries([mesh])