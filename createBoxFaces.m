% vertices.x = [20 20 20 20 -20 -20 -20 -20];
% vertices.y = [10 10 -10 -10 10 10 -10 -10];
% vertices.z = [10 -10 10 -10 10 -10 10 -10];
% vertices.faces = t_createBoxFaces(vertices)

function faces = createBoxFaces(vertices)
    if isstruct(vertices)
        vertices_mat = convertXYZstructToXYZmatrix(vertices);
    else
        vertices_mat = polygon_vertices;
    end
    
    % According to LiDAR coord.
    % Front
    [~, ind_max_x] = maxk(vertices_mat(1,:), 4);
    [faces(1).normal, faces(1).centroid] = computePlane(vertices_mat(:, ind_max_x));
    
    % Back
    [~, ind_min_x] = mink(vertices_mat(1,:), 4);
    [faces(2).normal, faces(2).centroid] = computePlane(vertices_mat(:, ind_min_x));
    
    % Left
    [~, ind_max_y] = maxk(vertices_mat(2,:), 4);
    [faces(3).normal, faces(3).centroid] = computePlane(vertices_mat(:, ind_max_y));
    
    % Right
    [~, ind_min_y] = mink(vertices_mat(2,:), 4);
    [faces(4).normal, faces(4).centroid] = computePlane(vertices_mat(:, ind_min_y));

    % Top
    [~, ind_max_z] = maxk(vertices_mat(3,:), 4);
    [faces(5).normal, faces(5).centroid] = computePlane(vertices_mat(:, ind_max_z));
    
    % Bottom
    [~, ind_min_z] = mink(vertices_mat(3,:), 4);
    [faces(6).normal, faces(6).centroid] = computePlane(vertices_mat(:, ind_min_z));
end