"""

下面的代码使用create_mesh_cubic，create_mesh_sphere和create_mesh_cylinder生成立方体，球体和圆柱体。
立方体涂成红色，球体涂成蓝色，圆柱体涂成绿色。 为两个网格计算法线以支持Phong阴影（请参见可视化3D网格和曲面法线估计）。
我们甚至可以使用create_mesh_coordinate_frame创建坐标轴，其原点设置为（-2，-2，-2）。

http://www.open3d.org/docs/release/tutorial/Basic/visualization.html#Geometry-primitives
"""

import open3d as o3d

print("Let's define some primitives")
# 立方体
mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
mesh_box.compute_vertex_normals()
mesh_box.paint_uniform_color([0.9, 0.1, 0.1])

# 球体
mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
mesh_sphere.compute_vertex_normals()
mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])

# 圆柱体
mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=4.0)
mesh_cylinder.compute_vertex_normals()
mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[-2, -2, -2])
"""
draw_geometries获取一个几何列表，并将它们一起渲染。 
另外，TriangleMesh支持+运算符以将多个网格合并为一个。 
我们建议使用第一种方法，因为它支持不同几何形状的组合（例如，可以与点云一起渲染网格）。
"""
print("We draw a few primitives using collection.")
o3d.visualization.draw_geometries(
        [mesh_box, mesh_sphere, mesh_cylinder, mesh_frame])

print("We draw a few primitives using + operator of mesh.")
o3d.visualization.draw_geometries(
        [mesh_box + mesh_sphere + mesh_cylinder + mesh_frame])