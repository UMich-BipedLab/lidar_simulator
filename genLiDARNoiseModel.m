function ring_noise_model = genLiDARNoiseModel(ring_number, LiDAR_opts)
    switch LiDAR_opts.properties.mechanics_noise_model
        case 0
            ring_noise_model = noSimpleMechanicalNoiseModel(ring_number, LiDAR_opts);
        case 1
            ring_noise_model = whiteNoise(ring_number, LiDAR_opts);
        case 2
            ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
                                                          LiDAR_opts);
%             for i = 1:32
%                 ring_number = i
%             ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
%                                                           LiDAR_opts)
%             end
    end
            
    
end