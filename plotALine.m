function plotALine(figure_handle, direction, point, start, end_)
    v = makeColumn(direction);
    t = linspace(start, end_, 2);
    e1 = [v(1) 0 0; 0 v(2) 0; 0 0 v(3)] * [t(1); t(1); t(1)] + makeColumn(point);
    e2 = [v(1) 0 0; 0 v(2) 0; 0 0 v(3)] * [t(end); t(end); t(end)] + makeColumn(point);
    e = [e1 e2];
    plot3(figure_handle, e(1,:), e(2,:), e(3,:))
end