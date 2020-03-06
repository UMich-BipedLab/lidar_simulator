function [normal, centroid, U] = computePlaneReturnR(object)
    if isstruct(object)
        if isfield(object, 'object_vertices')
            vertices_mat = convertXYZstructToXYZmatrix(object.object_vertices);
        else
            error("No 'ojbect_vertices' field in the input, Please check again.")
        end
    else
        vertices_mat = object;
    end

    if size(vertices_mat, 2) < size(vertices_mat, 1)
        vertices_mat = vertices_mat';
    end
    
    centroid = mean(vertices_mat, 2);
    [U, ~, ~] = svd(vertices_mat - centroid);
    normal = U(:, 3);
    normal = normal/norm(normal);       
    if normal(1) < 0
        normal = -normal;
    end
    centroid = makeColumn(centroid);
    normal = makeColumn(normal);
end