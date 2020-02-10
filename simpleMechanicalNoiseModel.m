function noise_model = simpleMechanicalNoiseModel(ring_num, LiDAR_opts)  
    
    % Ring number
    noise_model.ring = ring_num;
    
    % Range noise (+- 0.5 cm)
    range_max =  0.005;
    range_min = -range_max;
    noise_model.range_noise = genRandomNumber(range_min, range_max); 
    
    % Azimuth angle (+-20%: 0.04 deg <the resolution is around 0.2 deg>)
    az_noise_level = 0.2;
    az_max = az_noise_level * LiDAR_opts.properties.az_resolution;
    az_min = -az_max;
    noise_model.az_noise = genRandomNumber(az_max, az_min); 
    
    % Elevation angle (1 deg)
    % Becase matlab starts from 1, we have to minus 1 
    % Due to the dense region vs sparse region, 
    % we have to make sure the noise does not overlapped/ cross each other
    % To do this, we make the noise not exceed X% from the one before and
    % after.
    
    ring_num = ring_num - 1;
    num_ring = LiDAR_opts.properties.beam;
    
    el_noise_level = 1;
    noise_bound = 0.5; % 50% 
    el_noise = genRandomNumber(-el_noise_level, el_noise_level);
    strs = {(LiDAR_opts.properties.ordered_ring_elevation(:).ring)};
    index = find(ismember(strs, num2str(ring_num)));
    current_el = LiDAR_opts.properties.ordered_ring_elevation(index).angle;
    
    % Checking if exceed the noise_bound
    if index == 1
        % lowest beam
        if el_noise > 0
            upper_bound = noise_bound * ...
                          (LiDAR_opts.properties.ordered_ring_elevation(2).angle - current_el);
            el_noise = min(el_noise, upper_bound);
        end        
    elseif index == num_ring
        % toppest beam
        if el_noise < 0
            lower_bound = noise_bound * ...
                              (LiDAR_opts.properties.ordered_ring_elevation(num_ring-1).angle - current_el);
            el_noise = max(el_noise, lower_bound);
        end
    else
        upper_bound = noise_bound * ...
                          (LiDAR_opts.properties.ordered_ring_elevation(index+1).angle - current_el);
        lower_bound = noise_bound * ...
                          (LiDAR_opts.properties.ordered_ring_elevation(index-1).angle - current_el);
        el_noise = max(min(el_noise, upper_bound), lower_bound);
    end
    noise_model.el_noise = el_noise;
    
%     if ring_num == str2num(LiDAR_opts.properties.ordered_ring_elevation.ring)
%         % Lowest beam
%         elevation1 = LiDAR_opts.properties.ordered_ring_elevation(1).angle;
%         elevation2 = LiDAR_opts.properties.ordered_ring_elevation(2).angle/2;
%         noise_model.elevation = 
%     elseif ring_num == str2num(LiDAR_opts.properties.ordered_ring_elevation(num_beam).ring)
%         % Highest beam
%     else
%         
%     end
    
end
