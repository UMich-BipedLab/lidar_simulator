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
clear, clc
intrinsic_lib = '/home/brucebot/workspace/griztag/src/matlab/matlab/slider/3D_lidar_intrinsic_calibration';
noise_lib = "./noise";
utils = '/home/brucebot/workspace/griztag/src/matlab/matlab/slider/3D_lidar_intrinsic_calibration/matlab_utils';
scenes_dir = "./scenes";
methods = ["Lie", "BaseLine1","BaseLine3"];
save_path = "./results/noiseModel";


addpath(genpath(intrinsic_lib))
addpath(genpath(utils))
addpath(scenes_dir)
addpath(noise_lib)


R = rotx(0) * roty(0) * rotz(0);
T = [0, 0, 0];
pose = [R, T';zeros(1,3), 1];
LiDAR_opts.pose.H = pose;
LiDAR_opts.properties.array_distributions = ["Uniform-grid"];
LiDAR_opts.properties.array_distribution = 1;
LiDAR_opts.properties.diode_array = 20; 
LiDAR_opts.properties.h_coverage = 80;  % -80 ~ 80
LiDAR_opts.properties.v_coverage = 20;  % -80 ~ 80
LiDAR_opts.properties.array_size = 2;
LiDAR_opts.pose.range = 5;

LiDAR_opts.mechanism.types = ["rotating-head", "solid-state"];
LiDAR_opts.mechanism.type = 2;
LiDAR_opts.properties.mechanics_noise_model = 5;



% Create objects
scene = 1;
target_size = 6;
objects2 = genShape("polygon", target_size, 4);
object2_mat = convertXYZstructToXYZmatrix(objects2);
object2_mat_h = converToHomogeneousCoord(object2_mat);
rpy = [0 30 0]; % in degree
xyz = [3 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(1).size = target_size;

rpy = [10 10 0]; % in degree
xyz = [3 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(2).size = target_size;


rpy = [0 0 30]; % in degree
xyz = [3 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(3).size = target_size;

disp("Simulating...")
num_objects = size(objects, 2);
for i = 1:num_objects 
    [diodes_array(i).points, diodes_array_endpoints(i).points, diodes_array_endpoints_noisy(i).points] = simOPASSLiDAR(LiDAR_opts, objects(i));
end

disp("Done simulation")

% plotting
fig_handles = createFigHandleWithNumber(num_objects+2, 1, "training");
for i = 1:num_objects 
    hold(fig_handles(1), 'on')
    points = [diodes_array(i).points.diode.point];
    plotConnectedVerticesStructure(fig_handles(1), objects(i).object_vertices)
    scatter3(fig_handles(1), points(1,:), points(2,:), points(3,:), '.g')
    
    plotConnectedVerticesStructure(fig_handles(2), objects(i).object_vertices)
    scatter3(fig_handles(2), points(1,:), points(2,:), points(3,:), '.g')    
        
    plotConnectedVerticesStructure(fig_handles(2+i), objects(i).object_vertices)
%     scatter3(fig_handles(1), diodes_array_endpoints(i).points(1, :), diodes_array_endpoints(i).points(2, :), diodes_array_endpoints(i).points(3, :), '.r')
%     scatter3(fig_handles(1), diodes_array_endpoints_noisy(i).points(1, :), diodes_array_endpoints_noisy(i).points(2, :), diodes_array_endpoints_noisy(i).points(3, :), '.b')
end


viewCurrentPlot(fig_handles(1), [],[1,2])
% Parse points from simulator
num_grid_per_side = 8;
disp("Parsing data...")
for i = 1:num_objects 
    parsed_points(i).points = parsingPoints(diodes_array_endpoints_noisy(i).points, objects(i), num_grid_per_side);
end

for j = 1:num_objects
    for i = 1: size(parsed_points(j).points, 2)
        if isempty(parsed_points(j).points(i).points_mat)
            continue
        end
        scatter3(fig_handles(1), parsed_points(j).points(i).points_mat(1, :), parsed_points(j).points(i).points_mat(2, :), parsed_points(j).points(i).points_mat(3, :), '*')
        scatter3(fig_handles(2+j), parsed_points(j).points(i).points_mat(1, :), parsed_points(j).points(i).points_mat(2, :), parsed_points(j).points(i).points_mat(3, :), '*')
    end
    plotOriginalAxis(fig_handles(2+j), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)
    
end

plotOriginalAxis(fig_handles(2), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)
plotOriginalAxis(fig_handles(1), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)
% plotOriginalAxis(fig_handles(1), eye(4), LiDAR_opts.properties.array_size*0.5, '-k')
viewCurrentPlot(fig_handles(1), [],[-1,2])
disp("Done plotting ")

% Parse points for calibration structure
opts.datatype = "Simulation";
opts.num_beams = size(parsed_points(1).points, 2);
num_targets = num_objects;

for i = 1:num_targets
    data_split_with_ring_cartesian{i} = splitPointsBasedOnSegment(parsed_points(i).points, opts.num_beams, opts.datatype);
end

% Intrinsic Calibration
disp("Optimizing using Lie Group method...")
opt_formulation = ["Lie", "BaseLine1", "BaseLine2"]; % Lie or Spherical
opts.method = 1;
opts.num_scans = 1;
opts.num_iters = 1;
opt.check_rings = 0;
distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
distance(opts.num_iters).ring(opts.num_beams) = struct();
distance(opts.num_iters).mean = 0;

for k = 1: opts.num_iters
    fprintf("--- Working on %i/%i\n", k, opts.num_iters)
    [delta, plane, valid_rings_and_targets] = estimateIntrinsicLie(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring_cartesian, objects, opt.check_rings);
    if k == 1
        distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 
    end
    % update the corrected points
    data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring_cartesian, delta, valid_rings_and_targets, opt_formulation(opts.method));
    distance(k) = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 
end

% show numerical results
disp("Showing numerical results...")
disp("Showing current estimate")
results = struct('ring', {distance(end).ring(:).ring}, ...
                 'num_points', {distance(end).ring(:).num_points}, ...
                 'mean_original', {distance_original.ring(:).mean}, ...
                 'mean_calibrated', {distance(end).ring(:).mean}, ...
                 'mean_percentage', num2cell((abs([distance_original.ring(:).mean]) - abs([distance(end).ring(:).mean])) ./ abs([distance_original.ring(:).mean])), ...
                 'mean_diff_in_mm', num2cell(([distance_original.ring(:).mean] - [distance(end).ring(:).mean]) * 1e3), ...
                 'std_original', {distance_original.ring(:).std}, ...
                 'std_calibrated', {distance(end).ring(:).std}, ...
                 'std_diff', num2cell([distance_original.ring(:).std] - [distance(end).ring(:).std]), ...
                 'std_diff_in_mm', num2cell(([distance_original.ring(:).std] - [distance(end).ring(:).std])* 1e3));
struct2table(distance(end).ring(:))
disp("Showing comparison")
struct2table(results)

opts.save_path = "./results/";
if ~exist(opts.save_path, 'dir')
   mkdir(opts.save_path)
end
filename = opts.save_path + "sslidar-parameter" + num2str(scene) + ".mat";
save(filename, 'delta');

ds_orignal_mean_nonzeros = [distance_original.ring(:).mean];
mean_original =  abs(mean(ds_orignal_mean_nonzeros(find(ds_orignal_mean_nonzeros))))

ds_calibrated_mean_nonzeros = [distance(end).ring(:).mean];
mean_calibrated =  abs(mean(ds_calibrated_mean_nonzeros(find(ds_calibrated_mean_nonzeros))))
mean_percentage = (mean_original-mean_calibrated)/mean_original


%% Draw calibrated rings
for object = 1:length(objects)
    for ring = 1:opts.num_beams
        if isempty(data_split_with_ring_cartesian{object}(ring).points)
            continue;
        end
        % draw ring in differnt color
        offset_color = max(1, mod(object+1, length(objects(1))+1)); % 
        scatter3(fig_handles(1), data_split_with_ring_cartesian{object}(ring).points(1,:),...
                                 data_split_with_ring_cartesian{object}(ring).points(2,:),...
                                 data_split_with_ring_cartesian{object}(ring).points(3,:),...
                             50, 'r.')
        scatter3(fig_handles(2+object), data_split_with_ring_cartesian{object}(ring).points(1,:),...
                data_split_with_ring_cartesian{object}(ring).points(2,:),...
                data_split_with_ring_cartesian{object}(ring).points(3,:),...
                                     50, 'r.')
%         scatter3(fig_handles(4+object), data_split_with_ring_cartesian{object}(ring).points(1,:),...
%                              data_split_with_ring_cartesian{object}(ring).points(2,:),...
%                              data_split_with_ring_cartesian{object}(ring).points(3,:),...
%                              50, '.', 'MarkerEdgeColor', color_list{object})                   
%          text(fig_handles(1), mean(data_split_with_ring_cartesian{object}(ring).points(1,:)), ...
%                                      mean(data_split_with_ring_cartesian{object}(ring).points(2,:)), ...
%                                      mean(data_split_with_ring_cartesian{object}(ring).points(3,:)), "C"+num2str(ring-1))
    end
end


%% Validate
% cla(fig_handles(2))
R = rotx(0) * roty(0) * rotz(0);
T = [0, 0, 0];
pose = [R, T';zeros(1,3), 1];
LiDAR_opts.pose.H = pose;
LiDAR_opts.properties.array_distributions = ["Uniform-grid"];
LiDAR_opts.properties.array_distribution = 1;
LiDAR_opts.properties.diode_array = 20; 
LiDAR_opts.properties.h_coverage = 80;  % -80 ~ 80
LiDAR_opts.properties.v_coverage = 20;  % -80 ~ 80
LiDAR_opts.properties.array_size = 2;
LiDAR_opts.pose.range = 5;

LiDAR_opts.mechanism.types = ["rotating-head", "solid-state"];
LiDAR_opts.mechanism.type = 2;
LiDAR_opts.properties.mechanics_noise_model = 5;

scene = 1;
objects2 = genShape("polygon", target_size, 4);
object2_mat = convertXYZstructToXYZmatrix(objects2);
object2_mat_h = converToHomogeneousCoord(object2_mat);
rpy = [10 0 10]; % in degree
xyz = [5 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects_validation(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects_validation(1).size = target_size;

rpy = [10 0 -10]; % in degree
xyz = [5 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects_validation(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects_validation(2).size = target_size;


disp("Simulating...")
num_objects_v = size(objects_validation, 2);
% [diodes_array_v, diodes_array_endpoints_v, diodes_array_endpoints_noisy_v] = simOPASSLiDAR(LiDAR_opts, objects_validation(1));

fig_handles_v = createFigHandleWithNumber(num_objects_v+2, num_objects+3, "validatoin");

for i = 1:num_objects_v
    hold(fig_handles_v(1), 'on')
    [diodes_array_v(i).points, diodes_array_endpoints_v(i).points, diodes_array_endpoints_noisy_v(i).points] = simOPASSLiDAR(LiDAR_opts, objects_validation(i));
    points_v = [diodes_array_v(i).points.diode.point];
    plotConnectedVerticesStructure(fig_handles_v(1), objects_validation(i).object_vertices)
    plotConnectedVerticesStructure(fig_handles_v(2), objects_validation(i).object_vertices)
    plotConnectedVerticesStructure(fig_handles_v(2+i), objects_validation(i).object_vertices)
    
    plotOriginalAxis(fig_handles_v(2+i), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)
    viewCurrentPlot(fig_handles_v(2+i), [],[-50,36])
%     scatter3(fig_handles(2), points_v(1,:), points_v(2,:), points_v(3,:), '.g')
%     scatter3(fig_handles(2), diodes_array_endpoints_v(i).points(1, :), diodes_array_endpoints_v(i).points(2, :), diodes_array_endpoints_v(i).points(3, :), '.r')
%     scatter3(fig_handles(2), diodes_array_endpoints_noisy_v(i).points(1, :), diodes_array_endpoints_noisy_v(i).points(2, :), diodes_array_endpoints_noisy_v(i).points(3, :), '.b')
end
viewCurrentPlot(fig_handles_v(1), [],[-50,36])
plotOriginalAxis(fig_handles_v(1), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)
plotOriginalAxis(fig_handles_v(2), LiDAR_opts.pose.H, LiDAR_opts.properties.array_size*1)

% Parse points from simulator
num_grid_per_side = 8;
disp("Parsing data...")
for i = 1:num_objects_v
    parsed_points_v(i).points = parsingPoints(diodes_array_endpoints_noisy_v(i).points, objects_validation(i), num_grid_per_side);
end

for j = 1:num_objects_v
    for i = 1: size(parsed_points_v(j).points, 2)
        if isempty(parsed_points_v(j).points(i).points_mat)
            continue
        end
        scatter3(fig_handles_v(1), parsed_points_v(j).points(i).points_mat(1, :), parsed_points_v(j).points(i).points_mat(2, :), parsed_points_v(j).points(i).points_mat(3, :), '*')
        scatter3(fig_handles_v(2+j), parsed_points_v(j).points(i).points_mat(1, :), parsed_points_v(j).points(i).points_mat(2, :), parsed_points_v(j).points(i).points_mat(3, :), '*')
    end
end
disp("Done plotting ")


% Parse points for calibration structure
opts.datatype = "Simulation";
opts.num_beams = size(parsed_points_v(1).points, 2);
num_targets = num_objects_v;

for i = 1:num_targets
    data_split_with_ring_cartesian_v{i} = splitPointsBasedOnSegment(parsed_points_v(i).points, opts.num_beams, opts.datatype);
end

filename = opts.save_path + "sslidar-parameter" + num2str(scene) + ".mat";
trained_delta = load(filename, 'delta');
for i = 1:num_targets
    data_split_with_ring_cartesian_v_corrected{i} = correctSSLiDARPoints(data_split_with_ring_cartesian_v{i}, trained_delta);
end


% Draw Validation
for object = 1:length(objects_validation)
    [objects_validation(object).normals, objects_validation(object).centroid] = computePlane(objects_validation(object));
    objects_validation(object).unit_normals = objects_validation(object).normals/norm(objects_validation(object).normals);
    for ring = 1:opts.num_beams
        if isempty(data_split_with_ring_cartesian_v_corrected{object}(ring).points)
            continue;
        end
        scatter3(fig_handles_v(1), data_split_with_ring_cartesian_v_corrected{object}(ring).points(1,:),...
                                 data_split_with_ring_cartesian_v_corrected{object}(ring).points(2,:),...
                                 data_split_with_ring_cartesian_v_corrected{object}(ring).points(3,:),...
                             50, '.')        
         scatter3(fig_handles_v(2+object), data_split_with_ring_cartesian_v_corrected{object}(ring).points(1,:),...
         data_split_with_ring_cartesian_v_corrected{object}(ring).points(2,:),...
         data_split_with_ring_cartesian_v_corrected{object}(ring).points(3,:),...
         50, 'r.')
%          text(fig_handles(2), mean(data_split_with_ring_cartesian_v_corrected{object}(ring).points(1,:)), ...
%                                      mean(data_split_with_ring_cartesian_v_corrected{object}(ring).points(2,:)), ...
%                                      mean(data_split_with_ring_cartesian_v_corrected{object}(ring).points(3,:)), "C"+num2str(ring-1))
    end
end
distance_uncali= computePointsToPlaneDistance(data_split_with_ring_cartesian_v, objects_validation, opts.num_beams);
distance_v = computePointsToPlaneDistance(data_split_with_ring_cartesian_v_corrected, objects_validation, opts.num_beams);
struct2table(distance_v(end).ring(:))
distance_uncali.mean
distance_v.mean
(distance_uncali.mean - distance_v.mean)/distance_uncali.mean
% ds_orignal_mean_nonzeros = [distance_original.ring(:).mean];
% mean_original =  abs(mean(ds_orignal_mean_nonzeros(find(ds_orignal_mean_nonzeros))))
% 
% ds_calibrated_mean_nonzeros = [distance(end).ring(:).mean];
% mean_calibrated =  abs(mean(ds_calibrated_mean_nonzeros(find(ds_calibrated_mean_nonzeros))))
% mean_percentage = (mean_original-mean_calibrated)/mean_original

for i=1 : num_objects_v+2
    viewCurrentPlot(fig_handles_v(i), [],[-50,36])
    xlabel(fig_handles_v(i), "x")
    ylabel(fig_handles_v(i), "y")
    zlabel(fig_handles_v(i), "z")
end
for i=1 : num_objects+2
    viewCurrentPlot(fig_handles(i), [],[-50,36])
    xlabel(fig_handles(i), "x")
    ylabel(fig_handles(i), "y")
    zlabel(fig_handles(i), "z")
end
disp("done")