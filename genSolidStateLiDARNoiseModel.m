function ring_noise_model = genSolidStateLiDARNoiseModel(segmentation_type, LiDAR_opts)
    names_of_noise_models = ["noNoise",... 
                             "whiteNoise",...
                             "simpleBent", ...
                             "mirrorAngle"];
%     if isfield(segmentation_type, 'fist_time')
%         fprintf("--- Using %s noise model \n", names_of_noise_models(LiDAR_opts.properties.mechanics_noise_model+1));
%     end
    switch LiDAR_opts.properties.mechanics_noise_model
        case 0
            ring_noise_model = noNoise(segmentation_type);
        case 1
            ring_noise_model = whiteNoiseSolidState(segmentation_type);
        case 2
            ring_noise_model = returnPointFromUnitGrid(segmentation_type);
        case 3
            ring_noise_model = sinusoidalWaveNoise(segmentation_type);
        case 4
            ring_noise_model = simpleBent(segmentation_type, LiDAR_opts);
        case 5
            ring_noise_model = simpleBent2PlaneDirection(segmentation_type, LiDAR_opts);
        otherwise
            error("No such LiDAR noise model (Mechanism model): %i", ...
                   LiDAR_opts.properties.mechanics_noise_model)
    end
end