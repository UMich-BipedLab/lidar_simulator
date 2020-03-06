function [diodes_array, diodes_array_endpoints, diodes_array_endpoints_noisy] = simOPASSLiDAR(LiDAR_opts, object)
    diodes_array = genLiDARArray(LiDAR_opts);
    diodes_array_endpoints = zeros(4, LiDAR_opts.properties.diode_array^2);
    diodes_array_endpoints_noisy = zeros(4, LiDAR_opts.properties.diode_array^2);

    h_resolution = 2*LiDAR_opts.properties.h_coverage / LiDAR_opts.properties.diode_array;
    v_resolution = 2*LiDAR_opts.properties.v_coverage / LiDAR_opts.properties.diode_array;

    % parfor prepare
    num_points = (LiDAR_opts.properties.diode_array)^2;
    segmentation_type(num_points).angle.row = [];
    segmentation_type(num_points).angle.column = [];
    segmentation_type(num_points).angle.point = [];
    segmentation_type(num_points).angle.azimuth = [];
    segmentation_type(num_points).angle.elevation = [];
    segmentation_type(num_points).geom.objects = [];
    [normal, centroid] = computePlane(object);
    parfor point = 1:num_points
        [row, column] = findArrayToMatrixIndex(point, LiDAR_opts.properties.diode_array);
        % column should decide azimuth (z)
        % row should decide elevation  (y)
        azimuth  = LiDAR_opts.properties.h_coverage - h_resolution * column;
        elevation = LiDAR_opts.properties.v_coverage - v_resolution * row;
        start_point = diodes_array.diode(:, point).point;
        end_piont = measureSolidStateEndPoints(start_point, diodes_array.diode(:, point).T, [0 elevation azimuth], LiDAR_opts.pose.range);
        [diodes_array_endpoints(:, point), inside_polygon] = checkInsideInsidePolygonGivenPlaneAndLine(object, start_point, end_piont);
        segmentation_type(point).geom.objects = object;
        segmentation_type(point).angle.normals = normal;
        segmentation_type(point).angle.centroid = centroid;
        segmentation_type(point).angle.row = row;
        segmentation_type(point).angle.column = column;
        segmentation_type(point).angle.point = diodes_array_endpoints(:, point);
        segmentation_type(point).angle.azimuth = azimuth;
        segmentation_type(point).angle.elevation = elevation;
        noisy_model = genLiDARNoiseModel(segmentation_type(point), LiDAR_opts);
        %%% XXX Noise now only support when the array placed in x-axis
        diodes_array_endpoints_noisy(:, point) = [addTwoTypesOfNoise(diodes_array_endpoints(:, point), [], noisy_model);1];
    end
end