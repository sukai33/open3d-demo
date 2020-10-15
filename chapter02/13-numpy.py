import open3d as o3d
import numpy as np

# generate some neat n times 3 matrix using a variant of sync function
x = np.linspace(-3, 3, 401)
mesh_x, mesh_y = np.meshgrid(x, x)
z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
z_norm = (z - z.min()) / (z.max() - z.min())
xyz = np.zeros((np.size(mesh_x), 3))
xyz[:, 0] = np.reshape(mesh_x, -1)
xyz[:, 1] = np.reshape(mesh_y, -1)
xyz[:, 2] = np.reshape(z_norm, -1)
print('xyz')
print(xyz)

# From NumPy to open3d.PointCloud
"""
Open3D提供了从NumPy矩阵到3D向量的向量的转换。 通过使用Vector3dVector，可以将NumPy矩阵直接分配给open3d.PointCloud.points。
以这种方式，可以使用NumPy分配或修改任何类似的数据结构，例如open3d.PointCloud.colors或open3d.PointCloud.normals。 
下面的代码还将点云另存为ply文件，以供下一步使用。
"""

# Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
# o3d.io.write_point_cloud("../TestData/sync.ply", pcd)
o3d.visualization.draw_geometries([pcd])