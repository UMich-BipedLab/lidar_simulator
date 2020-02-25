function noise_model = simpleHomogeneousNoiseModelAddOnNoise(ring_num, LiDAR_opts)  
    noise_model = simpleMechanicalNoiseModel(ring_num,  LiDAR_opts);
    RT_mechanismThreeParams = mechanismThreeParamsToSim3(0, 0, noise_model.range_noise, noise_model.az_noise, noise_model.el_noise);
    noise_model.type = 'transformation';
    noise_model.sim3  = RT_mechanismThreeParams;
    noise_addon = rotx(1)*roty(2)*rotz(-1.5);% 10 degs
    noise_model.sim3(1:3,1:3) = RT_mechanismThreeParams(1:3, 1:3) * noise_addon;
end