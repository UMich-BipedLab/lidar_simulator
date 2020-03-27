function segmentation_type = simpleBent2PlaneDirection(segmentation_type, LiDAR_opts)
    %%% XXX Noise now only support when the array placed in x-axis
    
    segmentation_type.type = 'noisy_point';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            current_point = makeColumn(segmentation_type.angle.point(1:3));
            
            plane_normal = - segmentation_type.angle.normals;
            plane_centroid = segmentation_type.angle.centroid;
            sphere_distance = 10;
            sphere_centor = movePointGivenAVector(plane_centroid, plane_normal, sphere_distance);
            sphere_radious = 0.98 * sphere_distance;
            
            vec = sphere_centor - current_point;
            move_length = abs(norm(vec) - sphere_radious);
            moved_point = movePointGivenAVector(current_point, vec, move_length);
            segmentation_type.noisy_point = moved_point;
        end
    else
        error("This type is no yet supported")
    end
end