function points = makeColumn(points)
    if ~iscolumn(points)
        points = points';
    end
end