function noise_model = simpleHomogeneousNoiseModel(ring_num, LiDAR_opts)  
    noise_model = simpleMechanicalNoiseModel(ring_num,  LiDAR_opts);
    noise_model.type = 'transformation';
    noise_model.sim3 = mechanismThreeParamsToSim3(0, 0, noise_model.range_noise, noise_model.az_noise, noise_model.el_noise);
end
