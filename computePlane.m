function [normal, centroid] = computePlane(vertices_mat)
    centroid = mean(vertices_mat, 2);
    [U, ~, ~] = svd(vertices_mat - centroid);
    normal = U(:, 3);
    normal = normal/norm(normal);
end