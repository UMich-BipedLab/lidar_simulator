function noisy_point = addTwoTypesOfNoise(point, sensor_noise, model_noise)
    if model_noise.type == "additive"
        noisy_point = point + ...
                       [sensor_noise.rand_x + model_noise.x; ...
                        sensor_noise.rand_y + model_noise.y; ...
                        sensor_noise.rand_z + model_noise.z;
                        0; 0];
    elseif model_noise.type == "transformation"
        [~, point_h] = checkHomogeneousCoord(point(1:3));
        noisy_point = model_noise.sim3 * point_h;
        point(1:3) = noisy_point(1:3);
        noisy_point = point  + ...
                       [sensor_noise.rand_x; ...
                        sensor_noise.rand_y; ...
                        sensor_noise.rand_z;
                        0; 0];
    else
        error("No such noise type: %s, model_noise.type")
    end
end