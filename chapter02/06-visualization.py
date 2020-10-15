"""
http://www.open3d.org/docs/release/tutorial/Basic/visualization.html

Open3D提供了一个方便的可视化功能draw_geometries，该功能获取一系列几何对象（PointCloud，TriangleMesh或Image），并将它们一起呈现。
我们在可视化器中实现了许多功能，例如通过鼠标操作旋转，平移和缩放，更改渲染样式和屏幕捕获。 在窗口内按h可打印出完整的功能列表。

除了draw_geometries，Open3D还具有一组具有更高级功能的兄弟功能。 draw_geometries_with_custom_animation允许程序员定义自定义视图轨迹并在GUI中播放动画。
draw_geometries_with_animation_callback和draw_geometries_with_key_callback接受Python回调函数作为输入。
在自动动画循环中或在按键事件时调用回调函数。 有关详细信息，请参见自定义可视化。
"""
import open3d as o3d


print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("../TestData/fragment.ply")
o3d.visualization.draw_geometries([pcd])