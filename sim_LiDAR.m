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

% clc
% vertices_moved1.x = [9.8821 10.1710 10.2890 10.1179 9.8290 9.7110];
% vertices_moved1.y = [0.3157 -0.0816 -0.3972 -0.3157 0.0816 0.3972];
% vertices_moved1.z = [0.3694 0.4627 0.0933 -0.3694 -0.4627 -0.0933];
% 
% vertices_moved2.x = [4.8821 5.1710 5.2890 5.1179 4.8290 4.7110];
% vertices_moved2.y = [0.3750 2.6514e-17 -0.3750 -0.3750 -4.6414e-16 0.3750];
% vertices_moved2.z = [0.3090 0.4698 0.1609 -0.3090 -0.4698 -0.1609];
% % points = t_genPointsInPolygon(matrix);
% 
% 
% object_list = [vertices_moved1, vertices_moved2];
% 
% boundary.x = [20, -20];
% boundary.y = [20, -20];
% boundary.z = [20, -20];
% 
% LiDAR_opts.centriod = [0 0 0];
% LiDAR_opts.beam = 32;
% LiDAR_opts.az_resolution = 0.1; % in degree
% LiDAR_opts.elevation_struct = struct(...
%     'ring_0',  -25.0, ...   % 0
%     'ring_3',  -15.639, ... % 3
%     'ring_4',  -11.310, ... % 4
%     'ring_7',  -8.843, ...  % 7
%     'ring_8',  -7.254, ...  % 8
%     'ring_11', -6.148, ...  % 11
%     'ring_12', -5.333, ...  % 12
%     'ring_16', -4.667, ...  % 16
%     'ring_15', -4.0, ...    % 15
%     'ring_19', -3.667, ...  % 19
%     'ring_20', -3.333, ...  % 20
%     'ring_24', -3.0, ...    % 24
%     'ring_23', -2.667, ...  % 23
%     'ring_27', -2.333, ...  % 27
%     'ring_28', -2.0, ...    % 28
%     'ring_2',  -1.667, ...  % 2
%     'ring_31', -1.333, ...  % 31
%     'ring_1',  -1.0, ...    % 1
%     'ring_6',  -0.667, ...  % 6
%     'ring_10', -0.333, ...  % 10
%     'ring_5',   0.0, ...    % 5
%     'ring_9',   0.333, ...  % 9
%     'ring_14',  0.667, ...  % 14
%     'ring_18',  1.0, ...    % 18
%     'ring_13',  1.333, ...  % 13
%     'ring_17',  1.667, ...  % 17
%     'ring_22',  2.333, ...  % 22
%     'ring_21',  3.333, ...  % 21
%     'ring_26',  4.667, ...  % 26
%     'ring_25',  7.0, ...    % 25
%     'ring_30', 10.333, ...  % 30
%     'ring_29', 15.0 ...        % 29
% );
% 
% LiDAR_opts.range = 25;
% LiDAR_opts.ring_elevation = parseLiDARStruct(LiDAR_opts.elevation_struct, 'ring_', LiDAR_opts.beam);
% [object_list, LiDAR_ring_points, all_points]= t_sim_LiDAR(object_list, boundary, LiDAR_opts);
% 
% %%
% figure(999);
% fig = gca;
% scatter3(fig, all_points(1, :), all_points(2, :), all_points(3, :), 'b.')
% hold(fig, 'on')
% plotConnected3DVerticesStructure(fig, vertices_moved1, 'm')
% plotConnected3DVerticesStructure(fig, vertices_moved2, 'g')
% axis equal
% disp("Done")

function [objects, LiDAR_points, all_points] = sim_LiDAR(objects, boundary, LiDAR_opts)
    resolution = deg2rad(LiDAR_opts.properties.az_resolution);
    range = LiDAR_opts.properties.range;
    num_beam = LiDAR_opts.properties.beam;
    num_obj = length(objects);
    centroid = LiDAR_opts.centriod;
    
    if ~iscolumn(centroid)
        centroid = centroid';
    end
    
    num_points = floor(2*pi/resolution);
    ring_points(num_beam) = struct();
    ring_points(num_beam).points = [];
    
%     fig = figure(999);
%     cla(fig)

    % Initialization for points of a ring on objects
    parfor ring_num = 1:num_beam
        for obj = 1:num_obj
            LiDAR_points(ring_num).objects(obj).points.x = [];
            LiDAR_points(ring_num).objects(obj).points.y = [];
            LiDAR_points(ring_num).objects(obj).points.z = [];
            closest_point(ring_num).objects(obj).point = [];
            closest_point(ring_num).objects(obj).distance = [];
        end
        LiDAR_points(ring_num).points.x = [];
        LiDAR_points(ring_num).points.y = [];
        LiDAR_points(ring_num).points.z = [];
    end

    parfor ring_num = 1:num_beam
        LiDAR_opts_tmp =  LiDAR_opts; % make parfor more efficient
        elevation = deg2rad(LiDAR_opts_tmp.ring_elevation(ring_num).angle);
        points = zeros(3, num_points);

        for i = 1 : num_points
            azimuth = (i-1) * resolution;
%             fprintf("ring_num: %i; point_num: %i\n", ring_num, i)
            [x, y, z] = sph2cart(azimuth, elevation, range);
            point = [x; y; z];
            point = point + centroid; 
            % PS Boundaries limitation should be done after all the 
            % obstacles cheching
            
            % Fixed noise for this point
            rand_z = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(3), 1); % ring
            rand_y = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(2), 1); % noise on ring
            rand_x = getNoise(0, LiDAR_opts_tmp.properties.noise_sigma(1), 1); % noise on depth
            
            %% check with all objects
            flag_on_an_object = 0;
            for object = 1:num_obj
                % one point can only assign to one object (the closest
                % one). 
                % The distance is without noise (raw projection point)
                closest_point(ring_num).objects(object).distance = range;
                [point_on_plane, intersect] = findIntersectionOfPlaneAndLine(objects(object), centroid, point);
                if intersect == 1
                    [~, in] = checkInsidePolygon(objects(object), centroid, point_on_plane);
                    if in
                        flag_on_an_object = 1;
                        point_on_plane = limitInBoundary(point_on_plane, boundary); % check boundary
                        noisy_point_on_plane = point_on_plane + [rand_x; rand_y; rand_z]; % add noise
                        closest_point(ring_num).objects(object).distance = norm(point_on_plane);
                        closest_point(ring_num).objects(object).point = noisy_point_on_plane;
                    end
%                     in_polygon = inhull(I, objects(object));
                end
            end
            
            % Assigne the point to the closest one if happens
            if flag_on_an_object
                [~,  which_object] = min([closest_point(ring_num).objects(:).distance]);
                LiDAR_points(ring_num).objects(which_object).points.x = [LiDAR_points(ring_num).objects(which_object).points.x ...
                                                                         closest_point(ring_num).objects(which_object).point(1)];
                LiDAR_points(ring_num).objects(which_object).points.y = [LiDAR_points(ring_num).objects(which_object).points.y ...
                                                                         closest_point(ring_num).objects(which_object).point(2)];
                LiDAR_points(ring_num).objects(which_object).points.z = [LiDAR_points(ring_num).objects(which_object).points.z ...
                                                                         closest_point(ring_num).objects(which_object).point(3)];
                points(:, i) = closest_point(ring_num).objects(which_object).point;                                                      
            else
                point = limitInBoundary(point, boundary); % check boundary
                points(:, i) = point;
            end
%             scatter3(point(1), point(2), point(3))
%             hold on
%             drawnow
        end
%         pause
        LiDAR_points(ring_num).points.x = points(1, :);
        LiDAR_points(ring_num).points.y = points(2, :);
        LiDAR_points(ring_num).points.z = points(3, :);
    end
    
    % parse from lidar_points to objects
    parfor object = 1:num_obj
        for ring_num = 1:num_beam
            objects(object).ring_points(ring_num).x = LiDAR_points(ring_num).objects(object).points.x;
            objects(object).ring_points(ring_num).y = LiDAR_points(ring_num).objects(object).points.y;
            objects(object).ring_points(ring_num).z = LiDAR_points(ring_num).objects(object).points.z;
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





