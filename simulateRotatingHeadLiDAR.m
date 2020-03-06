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

function [objects, LiDAR_points, all_points] = simulateRotatingHeadLiDAR(objects, boundary, LiDAR_opts)
    resolution = deg2rad(LiDAR_opts.properties.az_resolution);
    range = LiDAR_opts.properties.range;
    num_beam = LiDAR_opts.properties.beam;
    num_obj = length(objects);
    LiDAR_centroid = LiDAR_opts.pose.centriod;
    LiDAR_pose = LiDAR_opts.pose.H;
    
    if ~iscolumn(LiDAR_centroid)
        LiDAR_centroid = LiDAR_centroid';
    end
    
    
    num_points = floor(2*pi/resolution);
    ring_points(num_beam) = struct();
    ring_points(num_beam).points = [];
    
%     fig = figure(999);
%     cla(fig)

    % Initialization for points of a ring on objects
    parfor ring_num = 1:num_beam
%     for ring_num = 1:num_beam
        for obj = 1:num_obj
            LiDAR_points(ring_num).objects(obj).points.x = [];
            LiDAR_points(ring_num).objects(obj).points.y = [];
            LiDAR_points(ring_num).objects(obj).points.z = [];
            LiDAR_points(ring_num).objects(obj).points.I = [];
            LiDAR_points(ring_num).objects(obj).points.R = [];
            LiDAR_points(ring_num).objects(obj).noiseless_points.x = [];
            LiDAR_points(ring_num).objects(obj).noiseless_points.y = [];
            LiDAR_points(ring_num).objects(obj).noiseless_points.z = [];
            LiDAR_points(ring_num).objects(obj).noiseless_points.I = [];
            LiDAR_points(ring_num).objects(obj).noiseless_points.R = [];
            closest_point(ring_num).objects(obj).point = [];
            closest_point(ring_num).objects(obj).noiseless_points = [];
            closest_point(ring_num).objects(obj).distance = [];
        end
        LiDAR_points(ring_num).points.x = [];
        LiDAR_points(ring_num).points.y = [];
        LiDAR_points(ring_num).points.z = [];
        LiDAR_points(ring_num).points.I = [];
        LiDAR_points(ring_num).points.R = [];
        LiDAR_points(ring_num).noise_model = genLiDARNoiseModel(ring_num, LiDAR_opts);
        
        LiDAR_points(ring_num).sensor_noise.rand_x = 0;
        LiDAR_points(ring_num).sensor_noise.rand_y = 0;
        LiDAR_points(ring_num).sensor_noise.rand_z = 0;
        
        LiDAR_points(ring_num).target_pose.xp = 0;
        LiDAR_points(ring_num).target_pose.xn = 0;
        LiDAR_points(ring_num).target_pose.yp = 0;
        LiDAR_points(ring_num).target_pose.yn = 0;
        LiDAR_points(ring_num).target_pose.zp = 0;
        LiDAR_points(ring_num).target_pose.zn = 0;
    end
    
%     for ring_num = 1:num_beam
    parfor ring_num = 1:num_beam
        LiDAR_opts_tmp =  LiDAR_opts; % make parfor more efficient
        % Noise from LiDAR's mechanical model for this ring
%         noise_range = LiDAR_points(ring_num).noise_model.range_noise;
%         noise_az = LiDAR_points(ring_num).noise_model.az_noise;
%         noise_el = LiDAR_points(ring_num).noise_model.el_noise;
%         [model_noisy_x, model_noisy_y, model_noisy_z] = sph2cart(noise_az, noise_el, noise_range);

         
        % Elevatoin angle and height for this beam
        elevation = deg2rad(LiDAR_opts_tmp.properties.ring_elevation(ring_num).angle);
        height = LiDAR_opts_tmp.properties.ring_elevation(ring_num).height; 
        points = zeros(5, num_points); % X Y Z I R
        
        for i = 1 : num_points
            azimuth = (i-1) * resolution; % in rad
            [x, y, z] = sph2cart(azimuth, elevation, range);
            z = z + height; % local frame offset
            point = [x; y; z; 1];
            
            % rotate points by lidar globle frame
            point = LiDAR_opts_tmp.pose.H * point; 
            point = limitInBoundaryWithBoundaryPlanes(point(1:3), LiDAR_centroid, boundary);

            % Apply model noise (Biases of the system)
%             [noisy_x, noisy_y noisy_z] = sph2cart(azimuth + deg2rad(noise_az), ...
%                                                    elevation + deg2rad(noise_el),...
%                                                    range + deg2rad(noise_range));
%             noisy_point = [noisy_x; noisy_y; noisy_z];
%             noisy_point = noisy_point + LiDAR_centroid; 
%             noisy_point = limitInBoundaryWithBoundaryPlanes(noisy_point, LiDAR_centroid, boundary);
                                                 
%             point = limitInBoundaryWithMaxMin(point, boundary); % check boundary

            
            %% Apply sensor noise (random noise)
            % Fixed noise for this point
            LiDAR_points(ring_num).sensor_noise.rand_z = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(3), 1); % ring
            LiDAR_points(ring_num).sensor_noise.rand_y = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(2), 1); % noise on ring
            LiDAR_points(ring_num).sensor_noise.rand_x = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(1), 1); % noise on depth
            
            %% check with all objects
            flag_on_an_object = 0;
            for object = 1:num_obj
                % one point can only assign to one object (the closest
                % one). 
                % The distance is without noise (raw projection point)
%                 closest_point(ring_num).objects(object).distance = range;
%                 [noiseless_point, status]= checkInsidePolygonGiven3DPoints(objects(object), LiDAR_centroid, point);
%                 if status
%                     closest_point(ring_num).objects(object).distance = noiseless_point.distance;
%                     
%                     % Apply noise from LiDAR's mechanical model and sensor
%                     % noise 
%                     noisy_point = [noiseless_point.point(1) + rand_x + model_noisy_x;...
%                                    noiseless_point.point(2) + rand_y + model_noisy_y;...
%                                    noiseless_point.point(3) + rand_z + model_noisy_z;...
%                                    255; ring_num];
%                     closest_point(ring_num).objects(object).point = noisy_point;
%                 end
                closest_point(ring_num).objects(object).distance = range;
                [point_on_plane, ~, intersect] = findIntersectionOfPlaneAndLine(objects(object), ...
                                                                                LiDAR_centroid, point);
                
                if intersect == 1
                    [~, in] = checkInsidePolygon(objects(object).object_vertices, point_on_plane);
                    if in
                        flag_on_an_object = 1;
%                         point_on_plane = limitInBoundaryWithMaxMin(point_on_plane, boundary); % check boundary
                        closest_point(ring_num).objects(object).distance = norm(point_on_plane);
                        
                        % Assign intensity and ring number
                        closest_point(ring_num).objects(object).noiseless_point = [point_on_plane; 255; ring_num];
                        
                        
                        % add noise
                        noisy_point_on_plane = addTwoTypesOfNoise(point_on_plane, ...
                                                                  LiDAR_points(ring_num).sensor_noise, ...
                                                                  LiDAR_points(ring_num).noise_model);
%                         noisy_point_on_plane = point_on_plane + ...
%                                                [rand_x + model_noisy_x; ...
%                                                 rand_y + model_noisy_y; ...
%                                                 rand_z + model_noisy_z;
%                                                 0; 0];  % intensity and ring number
                        closest_point(ring_num).objects(object).point = [noisy_point_on_plane; 255; ring_num];
                        
                        %assign the target pose to this ring
                        if objects(object).centroid(1)> 0.1
                            LiDAR_points(ring_num).target_pose.xp = 1;
                        elseif objects(object).centroid(1)< -0.1
                            LiDAR_points(ring_num).target_pose.xn = 1;
                        end
                        if objects(object).centroid(2)> 0.1
                            LiDAR_points(ring_num).target_pose.yp = 1;
                        elseif objects(object).centroid(2)< -0.1
                            LiDAR_points(ring_num).target_pose.yn = 1;
                        end
                        if objects(object).centroid'*objects(object).normal > 0
                            if objects(object).normal(3)> 0.1
                                LiDAR_points(ring_num).target_pose.zp = 1;
                            elseif objects(object).normal(3)< -0.1
                                LiDAR_points(ring_num).target_pose.zn = 1;
                            end
                        else 
                            if objects(object).normal(3)> 0.1
                                LiDAR_points(ring_num).target_pose.zn = 1;
                            elseif objects(object).normal(3)< -0.1
                                LiDAR_points(ring_num).target_pose.zp = 1;
                            end
                        end
                    end
%                     in_polygon = inhull(I, objects(object));
                end
            end
            
            % Assigne the point to the closest one if happens
            if flag_on_an_object && LiDAR_opts.properties.return_once
                [~,  which_object] = min([closest_point(ring_num).objects(:).distance]);
                current_point = closest_point(ring_num).objects(which_object).point;
                LiDAR_points(ring_num).objects(which_object).points.x = [LiDAR_points(ring_num).objects(which_object).points.x ...
                                                                         current_point(1)];
                LiDAR_points(ring_num).objects(which_object).points.y = [LiDAR_points(ring_num).objects(which_object).points.y ...
                                                                         current_point(2)];
                LiDAR_points(ring_num).objects(which_object).points.z = [LiDAR_points(ring_num).objects(which_object).points.z ...
                                                                         current_point(3)];
                LiDAR_points(ring_num).objects(which_object).points.I = [LiDAR_points(ring_num).objects(which_object).points.I ...
                                                                         current_point(4)];
                LiDAR_points(ring_num).objects(which_object).points.R = [LiDAR_points(ring_num).objects(which_object).points.R ...
                                                                         current_point(5)];
                % Noise-free points <Only used to compare against to it>
                current_noiseless_point = closest_point(ring_num).objects(which_object).noiseless_point;
                LiDAR_points(ring_num).objects(which_object).noiseless_points.x = [LiDAR_points(ring_num).objects(which_object).noiseless_points.x ...
                                                                         current_noiseless_point(1)];
                LiDAR_points(ring_num).objects(which_object).noiseless_points.y = [LiDAR_points(ring_num).objects(which_object).noiseless_points.y ...
                                                                         current_noiseless_point(2)];
                LiDAR_points(ring_num).objects(which_object).noiseless_points.z = [LiDAR_points(ring_num).objects(which_object).noiseless_points.z ...
                                                                         current_noiseless_point(3)];
                LiDAR_points(ring_num).objects(which_object).noiseless_points.I = [LiDAR_points(ring_num).objects(which_object).noiseless_points.I ...
                                                                         current_noiseless_point(4)];
                LiDAR_points(ring_num).objects(which_object).noiseless_points.R = [LiDAR_points(ring_num).objects(which_object).noiseless_points.R ...
                                                                         current_noiseless_point(5)];                                                     
                                                                     
                points(:, i) = current_point;
            elseif flag_on_an_object && ~LiDAR_opts.properties.return_once
                [~,  which_object] = min([closest_point(ring_num).objects(:).distance]);
                current_point = closest_point(ring_num).objects(which_object).point;
                points(:, i) = current_point;
                
                for object = 1:num_obj
                    if isempty(closest_point(ring_num).objects(object).point)
                        continue
                    end
                    LiDAR_points(ring_num).objects(object).points.x = [LiDAR_points(ring_num).objects(object).points.x, ...
                                                                       closest_point(ring_num).objects(object).point(1)];
                                                                   
                    LiDAR_points(ring_num).objects(object).points.y = [LiDAR_points(ring_num).objects(object).points.y, ...
                                                                       closest_point(ring_num).objects(object).point(2)];
                                                                   
                    LiDAR_points(ring_num).objects(object).points.z = [LiDAR_points(ring_num).objects(object).points.z, ...
                                                                       closest_point(ring_num).objects(object).point(3)];
                                                                   
                    LiDAR_points(ring_num).objects(object).points.I = [LiDAR_points(ring_num).objects(object).points.I, ...
                                                                       closest_point(ring_num).objects(object).point(4)];
                                                                   
                    LiDAR_points(ring_num).objects(object).points.R = [LiDAR_points(ring_num).objects(object).points.R, ...
                                                                       closest_point(ring_num).objects(object).point(5)];
                    
                    % Noise-free points <Only used to compare against to it>
                    LiDAR_points(ring_num).objects(object).noiseless_points.x = [LiDAR_points(ring_num).objects(object).noiseless_points.x, ...
                                                                                 closest_point(ring_num).objects(object).noiseless_point(1)];
                    LiDAR_points(ring_num).objects(object).noiseless_points.y = [LiDAR_points(ring_num).objects(object).noiseless_points.y, ...
                                                                                 closest_point(ring_num).objects(object).noiseless_point(2)];
                    LiDAR_points(ring_num).objects(object).noiseless_points.z = [LiDAR_points(ring_num).objects(object).noiseless_points.z, ...
                                                                                 closest_point(ring_num).objects(object).noiseless_point(3)];
                    LiDAR_points(ring_num).objects(object).noiseless_points.I = [LiDAR_points(ring_num).objects(object).noiseless_points.I, ...
                                                                                 closest_point(ring_num).objects(object).noiseless_point(4)];
                    LiDAR_points(ring_num).objects(object).noiseless_points.R = [LiDAR_points(ring_num).objects(object).noiseless_points.R, ...
                                                                                 closest_point(ring_num).objects(object).noiseless_point(5)];
                end
            else
%                 point = limitInBoundaryWithMaxMin(point, boundary); % check boundary
%                 point = limitInBoundaryWithBoundaryPlanes(point, LiDAR_centroid, boundary);
                noisy_point = addTwoTypesOfNoise(point, ...
                                                 LiDAR_points(ring_num).sensor_noise, ...
                                                 LiDAR_points(ring_num).noise_model);

                points(:, i) = [noisy_point; 255; ring_num];
            end
%             scatter3(point(1), point(2), point(3), '.')
%             hold on
            
        end
%         drawnow
%         pause
        LiDAR_points(ring_num).points.x = points(1, :);
        LiDAR_points(ring_num).points.y = points(2, :);
        LiDAR_points(ring_num).points.z = points(3, :);
        LiDAR_points(ring_num).points.I = points(4, :);
        LiDAR_points(ring_num).points.R = points(5, :);
    end
    
    % parse from lidar_points to objects
    for object = 1:num_obj
        objects(object).points_mat = [];
        for ring_num = 1:num_beam
            objects(object).ring_points(ring_num).x = LiDAR_points(ring_num).objects(object).points.x;
            objects(object).ring_points(ring_num).y = LiDAR_points(ring_num).objects(object).points.y;
            objects(object).ring_points(ring_num).z = LiDAR_points(ring_num).objects(object).points.z;
            objects(object).ring_points(ring_num).I = LiDAR_points(ring_num).objects(object).points.I;
            objects(object).ring_points(ring_num).R = LiDAR_points(ring_num).objects(object).points.R;

            points = [makeRow(LiDAR_points(ring_num).objects(object).points.x); ...
                      makeRow(LiDAR_points(ring_num).objects(object).points.y); ...
                      makeRow(LiDAR_points(ring_num).objects(object).points.z); ...
                      makeRow(LiDAR_points(ring_num).objects(object).points.I); ...
                      makeRow(LiDAR_points(ring_num).objects(object).points.R)];
            objects(object).points_mat = [objects(object).points_mat, points];
                        
            % Noise-free points <Only used to compare against to it>
            objects(object).noise_less_ring_points(ring_num).x = LiDAR_points(ring_num).objects(object).noiseless_points.x;
            objects(object).noise_less_ring_points(ring_num).y = LiDAR_points(ring_num).objects(object).noiseless_points.y;
            objects(object).noise_less_ring_points(ring_num).z = LiDAR_points(ring_num).objects(object).noiseless_points.z;
            objects(object).noise_less_ring_points(ring_num).I = LiDAR_points(ring_num).objects(object).noiseless_points.I;
            objects(object).noise_less_ring_points(ring_num).R = LiDAR_points(ring_num).objects(object).noiseless_points.R;
        end

    end
    
    % all points
    all_points = zeros(3, num_beam*num_points);
    for ring_num = 1:num_beam
        index = (ring_num - 1) * num_points + 1;
        all_points(:, index:index+num_points-1) = [LiDAR_points(ring_num).points.x; ...
                                                   LiDAR_points(ring_num).points.y; ...
                                                   LiDAR_points(ring_num).points.z];
    end
end