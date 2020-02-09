function point = makeColumn(point)
    if ~iscolumn(point)
        point = point';
    end
end