function segmentation_type = whiteNoiseSolidState(segmentation_type)
    %%% XXX Noise now only support when the array placed in x-axis
    segmentation_type.type = 'additive';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            segmentation_type.noisy_point = segmentation_type.angle.point;
            azimuth = segmentation_type.angle.azimuth;
            elevation = segmentation_type.angle.elevation;
            seed = abs(azimuth + elevation);
            
            %% x axis
            distance = norm(segmentation_type.angle.point);
            ratio = 0.02;
            noise_x = genRandomNumber(0, ratio*distance, seed, 1);
            
            %% y and z axis various by its angle
            ratio_y = 0.002;
            ratio_z = 0.002;

            noise_y = genRandomNumber(0, ratio_y * azimuth, seed, 1); % azimuth
            noise_z = genRandomNumber(0, ratio_z * elevation, seed, 1); % elevation
            noise = [noise_x noise_y noise_z];
        end
    else
        seed = 1;
        noise = genRandomNumber(0.01, 0.01, seed, 3);
    end
    
    segmentation_type.x = noise(1);
    segmentation_type.y = noise(2);
    segmentation_type.z = noise(3);
end