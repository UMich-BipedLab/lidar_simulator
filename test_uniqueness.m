%%
clear, clc
target_size = 10;
objects1 = genShape("polygon", target_size, 4);
object1_mat = convertXYZstructToXYZmatrix(objects1);
object1_mat_h = converToHomogeneousCoord(object1_mat);
rpy = [0 0 0]; % in degree
xyz = [3 0 0];
moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
objects(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
objects(1).points = [];


rpy = [0 0 90]; % in degree
xyz = [3 2 0];
moved_obj2_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
objects(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(2).points = [];

rpy = [0 45 45]; % in degree
xyz = [3 2 0];
moved_obj3_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
objects(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
objects(3).points = [];

fig_handles = createFigHandleWithNumber(1, 1, "test");
hold(fig_handles(1), 'on')
plotMultiplePolygonsVertices(fig_handles(1), objects)
plotOriginalAxis(fig_handles(1), eye(4))
[normal1, centroid1] = computePlane(objects(1));
[normal2, centroid2] = computePlane(objects(2));
[normal3, centroid3] = computePlane(objects(3));
[P1, L1, check1] = intersectPlane(normal1, centroid1 ,normal2, centroid2);
[P2, L2, check2] = intersectPlane(normal2, centroid2 ,normal3, centroid3);

plane_intersect_point = intersect3DLine(P1,L1,P2,L2);

ring_intersect_point = plane_intersect_point + [0;0;1];
az_res = deg2rad(0.2);
r = norm(ring_intersect_point);
el = asin(ring_intersect_point(3)/r);

num_point = (2*pi/az_res);
points = zeros(4, num_point);

objects;
for i = 1:num_point
    az = az_res * (i-1);
    [x, y, z] = sph2cart(az, el, r);
    points(:, i) = [x;y;z;1];
    for j = 1:size(objects,2)
        [point_in, inside_polygon] = checkInsideInsidePolygonGivenPlaneAndLine(objects(j), [0,0,0,0], [x, y, z,1]);
        if inside_polygon
            objects(j).points = [objects(j).points, makeColumn(point_in)];
        end
    end
end
%%
s = 3;
H = [s*eye(3) (1-s)*(plane_intersect_point+[0; 0; 0]); zeros(1,3), 1];

color_list = [".k", ".c", ".b"];
for i = 1:length(objects)
    if isempty(objects(i).points)
        continue
    end
    moved_points =  H * objects(i).points;
    scatter3(fig_handles(1), objects(i).points(1, :), objects(i).points(2, :), objects(i).points(3, :), color_list(i))
    scatter3(fig_handles(1), moved_points(1, :), moved_points(2, :), moved_points(3, :), 'r.')
end

scatter3(fig_handles(1), points(1, :), points(2, :), points(3, :), 'm.')
scatter3(fig_handles(1), plane_intersect_point(1), plane_intersect_point(2), plane_intersect_point(3), 'om', 'fill')
scatter3(fig_handles(1), ring_intersect_point(1), ring_intersect_point(2), ring_intersect_point(3), 'og', 'fill')
plotALine(fig_handles(1), L1, P1, -5, 5)
plotALine(fig_handles(1), L2, P2, 0, 10)
viewCurrentPlot(fig_handles(1), [],[-34,19])
disp("done")
