function [point, inside_polygon]= checkInsideInsidePolygonGivenPlaneAndLine(objects, start_point, end_piont)
    [point_on_plane, ~, intersect] = findIntersectionOfPlaneAndLine(objects, ...
                                                                    start_point(1:3), end_piont(1:3));
%     point_on_plane
    inside_polygon = 0;
    if intersect == 1
        [~, inside_polygon] = checkInsidePolygon(objects.object_vertices, point_on_plane);
        if inside_polygon
            point = [point_on_plane; 1];
        else
            point = end_piont;
        end
    else
        point = end_piont;
    end
end
