function moved_point = movePointGivenAVector(point, N, d)
    unit_N = N/norm(N);
    moved_point = [unit_N(1) 0 0; 0 unit_N(2) 0; 0 0 unit_N(3)] * d*ones(3,1) + makeColumn(point);
end