import open3d as o3d
from common import open3d_tutorial as o3dtut
from open3d.open3d_pybind.geometry import PointCloud, TriangleMesh, LineSet

# 凸壳 ----------------------------------------------------- Convex hull
mesh: TriangleMesh = o3dtut.get_bunny_mesh()
print(type(mesh))
# Function to sample points from the mesh, where each point has approximately the same distance to the neighbouring points (blue noise).
# 用于从网格采样点的功能，其中每个点到相邻点的距离大约相同（蓝噪声）
pcl: PointCloud = mesh.sample_points_poisson_disk(number_of_points=2000)
# 计算凸壳
hull, _ = pcl.compute_convex_hull() # hull -> TriangleMesh
hull_ls: LineSet = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))

o3d.visualization.draw_geometries([pcl, hull_ls])


