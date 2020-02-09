function angle_sum = computeAngleSum(object_vertices, point)
    n = length(object_vertices.x);
    angle_sum = 0;
    flag_on_a_node = 0;
    for i = 1:n
%         i
        p1(1) = object_vertices.x(i) - point(1);
        p1(2) = object_vertices.y(i) - point(2);
        p1(3) = object_vertices.z(i) - point(3);
        j = max(1, mod(i+1, n+1));
        p2(1) = object_vertices.x(j) - point(1);
        p2(2) = object_vertices.y(j) - point(2);
        p2(3) = object_vertices.z(j) - point(3);

        m1 = norm(p1);
        m2 = norm(p2);

        if m1*m2 <= 1e-5
            flag_on_a_node = 1; % We are on a node, consider this inside
            break;
        else
            costheta = dot(p1, p2) / (m1*m2);
        end
        angle_sum = angle_sum + acos(costheta);
    end
    
    if flag_on_a_node
        angle_sum = 2*pi;
    end
end