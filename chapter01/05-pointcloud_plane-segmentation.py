"""
http://www.open3d.org/docs/release/tutorial/Basic/pointcloud.html#Plane-segmentation

Open3D还包含使用RANSAC从点云中分割几何图元的支持。
要在点云中找到具有最大支持的平面，我们可以使用segement_plane该方法具有三个参数：

distance_threshold： 定义一个点到一个估计平面的最大距离，该点可被视为一个惯常值；
ransac_n： 定义随机采样的点数以估计一个平面；
num_iterations：定义对随机平面进行采样和验证的频率。

函数然后将平面返回为（a，b，c，d），这样对于平面上的每个点（x，y，z）都有ax + by + cz + d = 0。
该功能进一步调整内部点的索引列表。

"""
import open3d as o3d

pcd = o3d.io.read_point_cloud("../TestData/fragment.pcd")
# 分割平面
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# 根据索引列表提取点云
inlier_cloud = pcd.select_by_index(inliers)
# 给点云涂红色
inlier_cloud.paint_uniform_color([1.0, 0, 0])
# 根据索引列表，选出不属于inliers的点
outlier_cloud = pcd.select_by_index(inliers, invert=True)
# 绘制
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
