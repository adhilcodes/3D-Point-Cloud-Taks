import random
import math

class RANSAC():
    def __init__(self, point_cloud, max_iter, threshold):
        self.point_cloud = point_cloud
        self.max_iter = max_iter
        self.threshold = threshold

        inliers_result = set()
        while max_iter:
            max_iter -= 1
            # Add 3 random indexes
            random.seed()
            inliers = []
            while len(inliers) < 3:
                random_index = random.randint(0, len(self.point_cloud.X)-1)
                inliers.append(random_index)
            # print(inliers)

            try:
                # In case of *.xyz data
                x1, y1, z1, _, _, _ = point_cloud.loc[inliers[0]]
                x2, y2, z2, _, _, _ = point_cloud.loc[inliers[1]]
                x3, y3, z3, _, _, _ = point_cloud.loc[inliers[2]]
            except:
                # In case of *.pcd data
                x1, y1, z1 = point_cloud.loc[inliers[0]]
                x2, y2, z2 = point_cloud.loc[inliers[1]]
                x3, y3, z3 = point_cloud.loc[inliers[2]]
            # Plane Equation --> ax + by + cz + d = 0
            # Value of Constants for inlier plane
            a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a*x1 + b*y1 + c*z1)
            plane_lenght = max(0.1, math.sqrt(a*a + b*b + c*c))
