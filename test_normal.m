clc,
target_size = 5;
objects2 = genShape("polygon", target_size, 4);
object2_mat = convertXYZstructToXYZmatrix(objects2);
object2_mat_h = converToHomogeneousCoord(object2_mat);
rpy = [0 30 0]; % in degree
xyz = [3 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(1).size = target_size;
[N, C] = computePlane(objects(1));
d = 1;
moved_point = movePointGivenAVector(C, -N, d);


num_objects = size(objects, 2);
fig_handles = createFigHandleWithNumber(1, 1, "test");
for i = 1:num_objects
    hold(fig_handles(1), 'on')
    points = [scan(i).diodes_array.diode.point];
    plotConnectedVerticesStructure(fig_handles(1), objects(i).object_vertices)
    scatter3(fig_handles(1), C(1), C(2), C(3))
    scatter3(fig_handles(1), moved_point(1), moved_point(2), moved_point(3))
end
plotOriginalAxis(fig_handles(1), eye(4), 1*0.5, '-k')
viewCurrentPlot(fig_handles(1), [],[-16,8])

disp("done")