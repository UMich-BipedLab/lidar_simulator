function segmentation_type = simpleBent(segmentation_type, LiDAR_opts)
    %%% XXX Noise now only support when the array placed in x-axis
    
    segmentation_type.type = 'noisy_point';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            current_point = makeColumn(segmentation_type.angle.point(1:3));
            lidar_center = LiDAR_opts.pose.H(1:3,4);
            vec = lidar_center - current_point;
            
            sphere_radious = 0.95 * LiDAR_opts.pose.range;
            sphere_radious = 0.95 * 3;
            move_length = abs(norm(vec) - sphere_radious);
            moved_point = movePointGivenAVector(current_point, vec, move_length);
            segmentation_type.noisy_point = moved_point;
        end
    else
        error("This type is no yet supported")
    end
end