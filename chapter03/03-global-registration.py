"""
Both ICP registration and Colored point cloud registration are known as local registration methods because they rely on a rough alignment as initialization.
This tutorial shows another class of registration methods, known as global registration.
This family of algorithms do not require an alignment for initialization.
They usually produce less tight alignment results and are used as initialization of the local methods.

"""
import open3d as o3d
import numpy as np
import copy
from open3d.open3d_pybind.geometry import PointCloud

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

"""
Extract geometric feature
We down sample the point cloud, estimate normals, then compute a FPFH feature for each point. 
The FPFH feature is a 33-dimensional vector that describes the local geometric property of a point.
 A nearest neighbor query in the 33-dimensinal space can return points with similar local geometric structures. 
 See [Rasu2009] for details.
"""

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


"""
Input
This code below reads a source point cloud and a target point cloud from two files. 
They are misaligned with an identity matrix as transformation.
"""

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud("../TestData/ICP/cloud_bin_0.pcd")
    target = o3d.io.read_point_cloud("../TestData/ICP/cloud_bin_1.pcd")
    # source:PointCloud = o3d.io.read_point_cloud("../TestData/ss/s2.pcd")
    # target:PointCloud = o3d.io.read_point_cloud("../TestData/ss/s3.pcd")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                             [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    source.rotate(source.get_rotation_matrix_from_zyx((0, 0, 0)),
                  center=source.get_center())

    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


voxel_size = 0.05 # means 5cm for this dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

# RANSAC

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, # ransac_n
        [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], # checkers 对应点检查规则
        #  max_iteration: int = 1000, max_validation: int = 1000
        o3d.registration.RANSACConvergenceCriteria(4000000, 500)) # criteria 收敛标准
    return result

result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)