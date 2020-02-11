function points = makeRow(points)
    if ~isrow(points)
        points = points';
    end
end