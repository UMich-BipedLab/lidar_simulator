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

%validation
%General parameters
% clc
% 
% scene = 24; %training scene
% v_scene = 25; %validation scene
% methods = ["\Lie\", "\BaseLine1\", "\BaseLine3\"];
% % Intrinsic calibration 
% opts.method = 3; % Lie; BaseLine1; BaseLine3
% opts.datatype = "Simulation"; %Experiment , Simulation
% opts.iterative = 0;
% opts.show_results = 0;
% opts.show_statistics = 0;
% opts.noise_model = 2;
% opts.load_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\scene" + num2str(scene)+ methods(opts.method);
% opts.save_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\Validation25"+ "\scene" + num2str(scene)+ methods(opts.method);
% t_validation(scene, v_scene, opts)

function validation(t_scene, v_scene, opts, object_list, color_list, LiDAR_ring_points, LiDAR_opts)
    if ~exist(opts.save_path, 'dir')
       mkdir(opts.save_path)
    end
    % addpath('/home/brucebot/workspace/griztag/src/matlab/matlab/slider/intrinsic_latest')
    % opts.load_path = "./results/";

    filename = opts.load_path + "parameter" + num2str(t_scene) + ".mat";
    load(filename);
    fprintf("Calibration parameter from scene %i is loaded! \n", t_scene);

    opt_formulation = ["Lie","BaseLine1","BaseLine3"];
    opt_method = opt_formulation(opts.method);
    %% Create objects
%     disp("- Generating obstacles...")
%     [object_list, color_list] = CreateObstacles(v_scene);
% 
% 
    % Plotting parameters
    num_handles = length(object_list) + 5;
    start_number = 1;
    name = "testing";
    fig_handles = createFigHandleWithNumber(num_handles, start_number, name);

    % Plot all polygons
    plotMultiplePolygonsVertices(fig_handles(2), object_list, color_list)

    % Workspace boundary
    % boundary.x = [20, -20];
    % boundary.y = [10, -10];
    % boundary.z = [10, -10];
    boundary.x = [40, -40];
    boundary.y = [40, -40];
    boundary.z = [40, -40];
    boundary.vertices = createBoxVertices(boundary);
    boundary.faces = createBoxFaces(boundary.vertices);
    scatter3(fig_handles(2), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')

    plotOriginalAxis(fig_handles(2), eye(4),1)
    viewCurrentPlot(fig_handles(2), "3D environment (Scene " + num2str(v_scene) + ")")

    %% LiDAR properties
    disp("- Loading LiDAR properties...")
    %%% mechanics_noise_model
    % 0: no noise model 
    % 1: whiteNoise
    % 2: simpleMechanicalNoiseModel (3 params)
    % 3: complexMechanicalNoiseModel (6 params)
    % 4: simpleHomogeneousNoiseModel (use simpleMechanicalNoiseModel then convert to SE3)
    % 5: complexHomogeneousNoiseModel (use complexMechanicalNoiseModel then convert to SE3)
    % 6: simpleHomogeneousNoiseModelAddOnNoise (use simpleMechanicalNoiseModel then convert to SE3 and add on more noise)
%     LiDAR_opts.properties.mechanics_noise_model = opts.noise_model; 
%     LiDAR_opts.properties.sensor_noise_enable = 0;
%     LiDAR_opts.properties.rpm = 1200; % 300, 60, 900, 1200
%     LiDAR_opts.properties.range = 50;
%     LiDAR_opts.properties.return_once = 0;
%     LiDAR_opts.pose.centriod = [0 0 0];
%     LiDAR_opts.pose.rpy = [0 0 0]; % deg (roll pitch yaw)
%     LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);
%     LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
%     [LiDAR_opts.properties.ring_elevation, ...
%      LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties, 'ring_', LiDAR_opts.properties.beam);
% 
% 
%     %% Simulate environment
%     disp("- Simulating LiDAR environment given provided obstacles...")
%     [object_list, LiDAR_ring_points, all_points]= simulateLiDAR(object_list, boundary, LiDAR_opts);

    %% Calibrate point clouds
    
    disp("~ Calibrating LiDAR Point clouds")
    [object_list_calibrated, LiDAR_ring_points_calibrated] = simulateCalibratedLiDAR(object_list,LiDAR_ring_points,LiDAR_opts, delta, opt_method);

    %% Plotting simulation
    disp("- Drawing simulated LiDAR environment...")
    % scatter3(fig_handle(3), all_points(1, :), all_points(2, :), all_points(3, :), 'b.')
    % cla(fig_handle(3))

    for beam_num = 1:LiDAR_opts.properties.beam
        scatter3(fig_handles(3), LiDAR_ring_points(beam_num).points.x, ...
                                 LiDAR_ring_points(beam_num).points.y, ...
                                 LiDAR_ring_points(beam_num).points.z, '.')
    %     scatter3(fig_handles(3), LiDAR_ring_points(beam_num).calibrated_points.x, ...
    %                          LiDAR_ring_points(beam_num).calibrated_points.y, ...
    %                          LiDAR_ring_points(beam_num).calibrated_points.z, 'x')                     
        hold(fig_handles(3), 'on')
    %     text(fig_handles(3), max(LiDAR_ring_points(beam_num).points.x), ...
    %                          max(LiDAR_ring_points(beam_num).points.y), ...
    %                          max(LiDAR_ring_points(beam_num).points.z), num2str(beam_num))
    end


    plotMultiplePolygonsVertices(fig_handles(3), object_list_calibrated, color_list)
    plotOriginalAxis(fig_handles(3), LiDAR_opts.pose.H, 4, '-k')
    viewCurrentPlot(fig_handles(3), "LiDAR simulation (Scene " + num2str(v_scene) + ")")
    set(fig_handles(3), 'visible', 'off')
    set(fig_handles(3), 'Color', 'b')

    saveas(fig_handles(2),strcat(opts.save_path,'ValidateScene', num2str(1),'.fig'));
    saveas(fig_handles(3),strcat(opts.save_path,'ValidateScene', num2str(2),'.fig'));
    saveas(fig_handles(4),strcat(opts.save_path,'ValidateScene', num2str(3),'.fig'));
    % Plotting points on polygons
    % cla(fig_handle(4))
    disp("- Drawing points on obstacles...")
    plotMultiplePolygonsVertices(fig_handles(4), object_list_calibrated, color_list)
    % scatter3(fig_handle(4), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')
    plotOriginalAxis(fig_handles(3), LiDAR_opts.pose.H, 4, '-k')
    for object = 1:length(object_list)
        scatter3(fig_handles(4), [object_list(object).ring_points.x], ...
                                 [object_list(object).ring_points.y], ...
                                 [object_list(object).ring_points.z], '.', 'MarkerFaceColor',color_list{object})
        hold(fig_handles(4), 'on')
        scatter3(fig_handles(4), [object_list_calibrated(object).calibrated_ring_points.x], ...
                             [object_list_calibrated(object).calibrated_ring_points.y], ...
                             [object_list_calibrated(object).calibrated_ring_points.z], 'x', 'MarkerFaceColor',color_list{object})
        % Plot on separated plots
        % Noisy-points
        scatter3(fig_handles(4+object), [object_list(object).ring_points.x], ...
                                        [object_list(object).ring_points.y], ...
                                        [object_list(object).ring_points.z], '.', 'MarkerFaceColor', color_list{object})
        hold(fig_handles(4+object), 'on')
        scatter3(fig_handles(4+object), [object_list_calibrated(object).calibrated_ring_points.x], ...
                                        [object_list_calibrated(object).calibrated_ring_points.y], ...
                                        [object_list_calibrated(object).calibrated_ring_points.z], 'x', 'MarkerFaceColor', color_list{object})   

        hold(fig_handles(4+object), 'on')

        % Noise-less pionts
    %     scatter3(fig_handles(4+object), [object_list(object).noise_less_ring_points.x], ...
    %                                     [object_list(object).noise_less_ring_points.y], ...
    %                                     [object_list(object).noise_less_ring_points.z], '.y')
        for ring = 1:LiDAR_opts.properties.beam
            if isempty(object_list(object).ring_points(ring).x)
                continue;
            end

            text(fig_handles(4+object), mean([object_list(object).ring_points(ring).x]), ...
                                        mean([object_list(object).ring_points(ring).y]), ...
                                        mean([object_list(object).ring_points(ring).z]), "N-" + num2str(ring))
            % Noise-less 
    %         text(fig_handles(4+object), min([object_list(object).noise_less_ring_points(ring).x]), ...
    %                                     min([object_list(object).noise_less_ring_points(ring).y]), ...
    %                                     min([object_list(object).noise_less_ring_points(ring).z]), num2str(ring))
        end
        plotConnectedVerticesStructure(fig_handles(4+object), object_list(object).object_vertices, color_list{object})
    end
    view_angle = [-86, 14];
    viewCurrentPlot(fig_handles(4), "Rings on Objects (Scene " + num2str(v_scene) + ")", view_angle)
    view_angle = [90, 0];
    for object = 1:length(object_list)
        viewCurrentPlot(fig_handles(4+object), "Object 1 (Scene " + num2str(v_scene) + ")", view_angle)
        saveas(fig_handles(4+object),strcat(opts.save_path,'validateobj', num2str(object),'.fig'));
    end

    if opts.show_statistics
        fprintf("\n\n\n")
        disp("==================")
        disp("--- Statistics ---")
        disp("==================")
        fprintf("\n\n------------------------\n")
        disp("- std on each target")
        disp("------------------------")
        for object = 1:length(object_list)
            fprintf("\n--- Object %i\n", object)
            fprintf("std of x: %f\n", std([object_list(object).ring_points.x]))
            fprintf("std of y: %f\n", std([object_list(object).ring_points.y]))
            fprintf("std of z: %f\n", std([object_list(object).ring_points.z]))
        end

        fprintf("\n\n------------------------\n")
        disp("- Noise on each ring")
        disp("------------------------")
        struct2table([LiDAR_ring_points.noise_model])

    %     fprintf("\n\n------------------------\n")
    %     disp("- Numbers of points on each ring")
    %     disp("------------------------")
    %     for beam_num = 1:LiDAR_opts.properties.beam
    %         fprintf("\n--- ring %i\n", beam_num)
    %         fprintf("num_points of x: %i\n", length(LiDAR_ring_points(beam_num).points.x))
    %         fprintf("num_points of y: %i\n", length(LiDAR_ring_points(beam_num).points.y))
    %         fprintf("num_points of z: %i\n", length(LiDAR_ring_points(beam_num).points.z))
    %     end
    end

    %% Quantative results
    disp("Parsing data...")
    num_targets = length(object_list);
    data_split_with_ring_cartesian = cell(1,num_targets);
    calibrated_data_split_with_ring_cartesian = cell(1,num_targets);
    plane = cell(1,num_targets);

    for t = 1:length(object_list)
        data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(object_list(t).points_mat, LiDAR_opts.properties.beam, opts.datatype);
        calibrated_data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(object_list_calibrated(t).calibrated_points_mat, LiDAR_opts.properties.beam, opts.datatype);
        plane{t}.centroid =  [object_list(t).centroid; 1];
        plane{t}.normals =  object_list(t).normal;
        plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
    end 
    distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, LiDAR_opts.properties.beam, length(object_list));
    distance_calibrated =  point2PlaneDistance(calibrated_data_split_with_ring_cartesian, plane, LiDAR_opts.properties.beam, length(object_list));
    fprintf("distance_original is %i m\n", distance_original.mean);
    fprintf("distance_calibrated is %i m\n", distance_calibrated.mean);

    disp("Showing numerical results...")
    disp("Showing current estimate")
    results = struct('ring', {distance_original(1).ring(:).ring}, ...
                     'num_points', {distance_original(1).ring(:).num_points}, ...
                     'mean_original', {distance_original(1).ring(:).mean}, ...
                     'mean_calibrated', {distance_calibrated(1).ring(:).mean}, ...
                     'mean_diff', num2cell([distance_original(1).ring(:).mean] - [distance_calibrated(1).ring(:).mean]), ...
                     'mean_percentage', num2cell((abs([distance_original(1).ring(:).mean]) - abs([distance_calibrated(1).ring(:).mean])) ./ abs([distance_original(1).ring(:).mean])));
    struct2table(distance_calibrated(1).ring(:))
    disp("Showing comparison")
    struct2table(results)
    save(opts.save_path + "data.mat", 'results');
    % [[results(1:16).mean_calibrated]; [results(17:32).mean_calibrated]]
end