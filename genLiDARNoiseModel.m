function ring_noise_model = genLiDARNoiseModel(ring_number, LiDAR_opts)
    switch LiDAR_opts.noise_model
        case 1
            ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
                                                          LiDAR_opts);
%             for i = 1:32
%                 ring_number = i
%             ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
%                                                           LiDAR_opts)
%             end
    end
            
    
end