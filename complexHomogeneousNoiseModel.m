function noise_model = complexHomogeneousNoiseModel(ring_num, LiDAR_opts)  
    noise_model = complexMechanicalNoiseModel(ring_num,  LiDAR_opts);
    noise_model.type = 'transformation';
    noise_model.sim3 = mechanismSixParamsToSim3(0, 0, noise_model.range_noise, ...
                             (noise_model.az_noise), (noise_model.el_noise),...
                             noise_model.range_scaling, noise_model.h_offset, noise_model.v_offset);
end
