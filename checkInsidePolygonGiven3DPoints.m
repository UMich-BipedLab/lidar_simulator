function [point, status]= checkInsidePolygonGiven3DPoints(object, LiDAR_centroid, point_3D)
    point = [];
    status = 0; % not in by default
    [point_on_plane, ~, intersect] = findIntersectionOfPlaneAndLine(object, ...
                                                                    LiDAR_centroid, point_3D);
    if intersect == 1
        [~, in] = checkInsidePolygon(object.object_vertices, point_on_plane);
        if in
            status = 1;
%                         point_on_plane = limitInBoundaryWithMaxMin(point_on_plane, boundary); % check boundary
            point.distance = norm(point_on_plane);

            % Assign intensity and ring number
%             point_on_plane = [point_on_plane; 255; ring_num];
            point.point = point_on_plane;
        end
%         in_polygon = inhull(I, objects(object));
    end
end