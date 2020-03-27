function segmentation_type = noNoise(segmentation_type)
    segmentation_type.type = 'additive';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            segmentation_type.noisy_point = segmentation_type.angle.point;
            segmentation_type.x = 0;
            segmentation_type.y = 0;
            segmentation_type.z = 0;
        end
    else
        segmentation_type.x = 0;
        segmentation_type.y = 0;
        segmentation_type.z = 0;
    end
end