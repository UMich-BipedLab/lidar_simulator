function noisy_point = addTwoTypesOfNoise(point, sensor_noise, noisy_model)
    if isempty(sensor_noise)
        sensor_noise.rand_x = 0;
        sensor_noise.rand_y = 0;
        sensor_noise.rand_z = 0;
    end
    if noisy_model.type == "additive"
        noisy_point = point(1:3) + ...
                       [sensor_noise.rand_x + noisy_model.x; ...
                        sensor_noise.rand_y + noisy_model.y; ...
                        sensor_noise.rand_z + noisy_model.z;];
    elseif noisy_model.type =="subtraction"
        noisy_point = point(1:3) + ...
                       [sensor_noise.rand_x - noisy_model.x; ...
                        sensor_noise.rand_y - noisy_model.y; ...
                        sensor_noise.rand_z - noisy_model.z;];
    elseif noisy_model.type == "transformation"
        [~, point_h] = checkHomogeneousCoord(point(1:3));
        if isfield(noisy_model, 'sim3')
            noisy_point = noisy_model.sim3 * point_h;
            noisy_point = noisy_point(1:3)  + ...
                           [sensor_noise.rand_x; ...
                            sensor_noise.rand_y; ...
                            sensor_noise.rand_z;];
        else
            error("No sim3 in the struct")
        end
    elseif noisy_model.type == "noisy_point"
        noisy_point = noisy_model.noisy_point;
    else
        error("No such noise type: %s, model_noise.type")
    end
end