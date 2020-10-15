import open3d as o3d

if __name__ == '__main__':
    # help(o3d)
    # help(o3d.geometry.PointCloud)
    help(o3d.io.read_point_cloud)

    pcd = o3d.io.read_point_cloud("./TestData/fragment.pcd")
    print(pcd)

    rst = o3d.io.write_point_cloud("./output/copy_of_fragment.pcd", pcd)

    print("write result: ", rst)
