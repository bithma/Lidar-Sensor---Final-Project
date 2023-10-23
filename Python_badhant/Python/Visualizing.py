import numpy as np
import open3d as o3d

point_cloud = o3d.io.read_point_cloud("number_measurements.xyz",format='xyz')
print(point_cloud)


o3d.visualization.draw_geometries([point_cloud])


point = 1
lines = []

for a in range(10):
    for x in range(63):
        point+=1
        lines.append([point,(point+1)])

point = 1
steps = 64

for b in range(10):
    for y in range(63):
        point += 1
        lines.append([point,point+steps])

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])
