import numpy as np
import open3d as o3d


# OPENING THE POINT CLOUD
print("Loading point cloud")
path = "/home/adhil/Desktop/Lab/Point Cloud Segmentation/Data/0000000004.pcd"


# VISUALIZING THE POINT CLOUD
pcd = o3d.io.read_point_cloud(path)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

# RANSAC

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

inlier_cloud = pcd.select_by_index(inliers)
outlier_cloud = pcd.select_by_index(inliers, invert=True)

inlier_cloud.paint_uniform_color([1, 0, 0])
outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
