%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function noise_model = simpleMechanicalNoiseModel(ring_num, LiDAR_opts)  
    
    % Ring number
    noise_model.ring = ring_num;
    noise_model.type = 'additive';
    
    % Range noise (+- 0.5 cm)
    range_max =  0.05;
%     range_max =  0.1;
    range_min = -range_max;
    noise_model.range_noise = genRandomNumber(range_min, range_max, ring_num); 
    
    % Azimuth angle (+-20%: 0.04 deg <the resolution is around 0.2 deg>)
    az_noise_level = 0.2;
%     az_noise_level = 1;
    az_max = az_noise_level * LiDAR_opts.properties.az_resolution;
    az_min = -az_max;
    noise_model.az_noise = genRandomNumber(az_max, az_min, ring_num); 
    
    % Elevation angle (1 deg)
    % Becase matlab starts from 1, we have to minus 1 
    % Due to the dense region vs sparse region, 
    % we have to make sure the noise does not overlapped/ cross each other
    % To do this, we make the noise not exceed X% from the one before and
    % after.
    
    ring_num = ring_num - 1;
    num_ring = LiDAR_opts.properties.beam;
    
    el_noise_level = 0.2;
%     el_noise_level = 5;
    noise_bound = 0.5; % 50% 
    el_noise = genRandomNumber(-el_noise_level, el_noise_level, ring_num);
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
    [noise_model.x, noise_model.y, noise_model.z] = sph2cart(deg2rad(noise_model.az_noise), deg2rad(noise_model.el_noise), noise_model.range_noise);
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
