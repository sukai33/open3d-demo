"""
要绘制线，必须定义LineSet并创建一组点和一组边。 边是一对点索引。
上面的示例创建了自定义的点和边（表示为线）以制作立方体。
颜色是可选的-在此示例中，将红色[1,0,0]分配给每个边缘。

"""

import open3d as o3d

print("Let's draw a cubic using o3d.geometry.LineSet.")
points = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
]
lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
]
colors = [[1, 0, 0] for i in range(len(lines))]

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([line_set])