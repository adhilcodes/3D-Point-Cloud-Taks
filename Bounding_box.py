import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


# OPENING THE POINT CLOUD
print("Loading point cloud")
path = "./Data/0000000003.pcd"


# VISUALIZING THE POINT CLOUD
pcd = o3d.io.read_point_cloud(path)
print(pcd)
print(np.asarray(pcd.points))
#o3d.visualization.draw_geometries([pcd])

## VOXEL GRID DOWNSAMPLING

voxel = pcd.voxel_down_sample(voxel_size=0.1)
print(f"Points before downsampling: {len(pcd.points)}", pcd)
print(f"Points after downsampling: {len(pcd.points)}", voxel)

#o3d.visualization.draw_geometries([voxel])


# RANSAC

plane_model, inliers = voxel.segment_plane(distance_threshold=0.09, ransac_n=3, num_iterations=1000)

inlier_cloud = voxel.select_by_index(inliers)
outlier_cloud = voxel.select_by_index(inliers, invert=True)

inlier_cloud.paint_uniform_color([1, 0, 0])
outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

#o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

'''
# CLUSTERING USING DBSCAN


labels = np.array(voxel.cluster_dbscan(eps=0.02, min_points=10))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

colors = plt.get_cmap("tab20")(labels / (max_label 
if max_label > 0 else 1))
colors[labels < 0] = 0
voxel.colors = o3d.utility.Vector3dVector(colors[:, :3])
#o3d.visualization.draw_geometries([voxel])
'''

# 3D BOUNDING BOX

aabox = pcd.get_oriented_bounding_box()
aabox.color = (1, 0, 0)

o3d.visualization.draw_geometries([pcd, aabox])

# get rotation matrix for truck
R = truck.get_rotation_matrix_from_zxy((0, np.pi/2, 0))
xyz = truck.rotate(R, center= (0,0,0))
obb_ = truck.get_oriented_bounding_box()
obb_.color = (1,0,1)

truck.paint_uniform_color((0,0,1))
o3d.visualization.draw_geometries([pcd,obb_])
