function data = correctSSLiDARPoints(data, trained_delta)
    for i = 1: size(data, 2)
        if isempty(data(i).points)
            continue
        end
        patch_points = data(i).points;
        if trained_delta.delta(i).Affine == eye(4) % remove if no calibration parameters exists
            data(i).points = [];
        else
            data(i).points = trained_delta.delta(i).Affine * patch_points;
        end
    end
end