clc, clear
point = [0 2 3];
division = 4;
object.size = 5;

% t_returnTransformFromUnitGrid(point, division, object)

% function [] = t_returnTransformFromUnitGrid(point, division, object)
cla
point_unit = point ./ object.size;
    
%     
%     
%     R = 2;
%     [Y,Z] = meshgrid(-0.5:1/division:0.5);
%     X = sqrt(R.^2 - Z.^2 - Y.^2);
% %     Z(imag(Z) ~= 0) = 0;
%     mesh(X, Z, Y);
% %     [x, y, z] = sphere(division);
% %     x = 0.5 * x;
% %     y = 0.5 * y;
% %     z = 0.5 * z;
% %     x = x()
% %     surf(x,y,z)
[x, y, z] = sphere(division);               %# Makes a 21-by-21 point sphere
x = x(division:end,:);                %# Keep top 11 x points
y = y(division:end,:);                %# Keep top 11 y points
z = z(division:end,:);                %# Keep top 11 z points
z = z - z(1,1); % move to (x, y, 0)
r = 2;                          %# A radius value
a = gradient(z);
hs = surf(r.*x,r.*y,r.*z, 'AlphaData',a, 'FaceAlpha','flat',...
    'FaceColor','blue');      %# Plot the surface
% hs.AlphaData = 0.5;
% hs2 = surf(r.*x,r.*y,r.*z);      %# Plot the surface
direction = [0 0 1];            % Specify Direction
rotate(hs, direction, 180/division)       % Rotate The Object (Hemisphere) in ‘Direction’ By 90°
% rotate(hs2, direction, 45)       % Rotate The Object (Hemisphere) in ‘Direction’ By 90°
% rotate(hs2, [0 1 0], 90)       % Rotate The Object (Hemisphere) in ‘Direction’ By 90°
axis equal;                     %# Make the scaling on the x, y, and z axes equal
grid on
plane(division).vertices = [];
point_xy = [point_unit(3), point_unit(2), point_unit(1)]
XData = hs.XData;
YData = hs.YData;
ZData = hs.ZData;

%%
for i = 1:division
    i
%     j = mod(j, 4);
    plane(i).object_vertices.x = [0 XData(1, i), XData(1, i+1)];
    plane(i).object_vertices.y = [0 YData(1, i), YData(1, i+1)];
    plane(i).object_vertices.z = [ZData(2,i) ZData(1, i), ZData(1, i+1)];
    
    plane(i).triangle.x = [0 XData(1, i), XData(1, i+1)];
    plane(i).triangle.y = [0 YData(1, i), YData(1, i+1)];
    plane(i).triangle.z = [0 ZData(1, i), ZData(1, i+1)];
    [plane(i).normal, plane(i).centroid, plane(i).R] = computePlaneReturnR(plane(i));
    scatter3(plane(i).triangle.x, plane(i).triangle.y, plane(i).triangle.z, 'y', 'fill')
    plane(i).angle_sum = computeAngleSum(plane(i).triangle, point_xy); 
    plane(i).status = 0;
    if abs(plane(i).angle_sum - 2*pi) < 1e-5
        plane(i).status = 1;
        interior = point;
    end
end
% scatter3(plane(1).object_vertices.x, plane(1).object_vertices.y, plane(1).object_vertices.z, 'o')
% scatter3(plane(2).object_vertices.x, plane(2).object_vertices.y, plane(2).object_vertices.z, 'd')
% scatter3(plane(3).object_vertices.x, plane(3).object_vertices.y, plane(3).object_vertices.z, '*')
% scatter3(plane(4).object_vertices.x, plane(4).object_vertices.y, plane(4).object_vertices.z, 'x')

[select_angle, which_polygon] = min(abs([plane(:).angle_sum]- 2*pi))
% H = eye(4);
% H(1:3,1:3) = (plane(which_polygon).R)';
% new_point = H * makeColumn([point_unit 1]);
[new_point, ~, intersect] = findIntersectionOfPlaneAndLine(plane(which_polygon), ...
                                                            point_xy(1:3), point_xy + [0 0 1])
scatter3(new_point(1), new_point(2), new_point(3), 'r', 'fill')
scatter3(point_xy(1), point_xy(2), point_xy(3), 'g', 'fill')
disp("done")
% end