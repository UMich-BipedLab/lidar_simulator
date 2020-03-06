function parsed_points_t = parsingPoints(points, object, num_grid_per_side)
    points = makeWideMatrix(points);
    num_points = size(points, 2);
    top_left = [max(object.object_vertices.x);
                max(object.object_vertices.y)
                max(object.object_vertices.z)];
    top_right = [max(object.object_vertices.x);
                 min(object.object_vertices.y)
                 max(object.object_vertices.z)];
    bottom_left = [max(object.object_vertices.x);
                   max(object.object_vertices.y)
                   min(object.object_vertices.z)];
    v1 = top_right - top_left;
    v1 = v1 / norm(v1);
    v2 = bottom_left - top_left;
    v2 = v2 / norm(v2);
    grid_length = object.size/num_grid_per_side;
    
    parsed_points_t(num_grid_per_side^2).points_mat = [];
    for i = 1:num_points
       v = points(1:3, i) - top_left;
       cell_column = ceil((dot(v, v1))/grid_length);
       cell_row = ceil((dot(v, v2))/grid_length);
       if cell_column<=0 || cell_row<=0 || cell_column>num_grid_per_side || cell_row>num_grid_per_side
           continue
       end
       index = cell_column + num_grid_per_side * (cell_row-1);
       parsed_points_t(index).points_mat = ...
           [parsed_points_t(index).points_mat, [points(1:3, i); 255; index]];
   end
end 