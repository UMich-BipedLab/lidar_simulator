function intersect_point = intersect3DLine(a,v1,b,v2)
%     vec = (b-a);
    a = makeColumn(a);
    b = makeColumn(b);
    v1 = makeColumn(v1);
    v2 = makeColumn(v2);

    f = v1; e = v2; g = b-a;
    direc = dot(cross(f,g)/(norm(cross(f,g))), cross(f,e)/(norm(cross(f,e))));
    intersect_point = b - direc * norm(cross(f, g))/norm(cross(f,e)) * e;
end