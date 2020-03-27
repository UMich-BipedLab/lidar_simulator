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


function [object_list, LiDAR_ring_points, all_points, LiDAR_opts] = simuateLiDAREnvironment(scene, opts)
    %% Create objects
    disp("- Generating obstacles...")
    [object_list, color_list] = CreateObstacles(scene);

    % Plotting parameters
    num_handles = length(object_list) + 5;
    figure_start_number = 1;
    figure_name = "testing";
    fig_handles = createFigHandleWithNumber(num_handles, figure_start_number, figure_name);
    plotMultiplePolygonsVertices(fig_handles(2), object_list, color_list)
    plotOriginalAxis(fig_handles(2), eye(4), 1)
    
    % Workspace boundary
    boundary.x = [40, -40];
    boundary.y = [40, -40];
    boundary.z = [40, -40];
    boundary.vertices = createBoxVertices(boundary);
    boundary.faces = createBoxFaces(boundary.vertices);
    scatter3(fig_handles(2), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')
    viewCurrentPlot(fig_handles(2), "3D environment (Scene " + num2str(scene) + ")")


    %% LiDAR properties
    disp("- Loading LiDAR properties...")
    LiDAR_opts.mechanism.types = ["rotating-head", "solid-state"];
    LiDAR_opts.mechanism.type = opts.lidar.type; 

    LiDAR_opts.properties.range = 30;
    LiDAR_opts.properties.return_once = 0;
    LiDAR_opts.pose.centriod = [0 0 0];
    LiDAR_opts.pose.rpy = [0 0 0]; % deg (roll pitch yaw)
    LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);


    if LiDAR_opts.mechanism.types(LiDAR_opts.mechanism.type) == "rotating-head"
        %%% mechanics_noise_model (rotating-head types)
        % 0: no noise model 
        % 1: whiteNoise
        % 2: simpleMechanicalNoiseModel (3 params)
        % 3: complexMechanicalNoiseModel (6 params)
        % 4: simpleHomogeneousNoiseModel (use simpleMechanicalNoiseModel then convert to SE3)
        % 5: [BUGGY] complexHomogeneousNoiseModel (use complexMechanicalNoiseModel then convert to SE3)
        % 6: simpleHomogeneousNoiseModelAddOnNoise (use simpleMechanicalNoiseModel then convert to SE3 and add on more noise)
        LiDAR_opts.properties.mechanics_noise_model = opts.mechanics_noise_model; 
        LiDAR_opts.properties.sensor_noise_enable = opts.sensor_noise_enable;
        LiDAR_opts.properties.rpm = 1200; % 300, 600, 900, 1200
        LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
        [LiDAR_opts.properties.ring_elevation, ...
        LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties, 'ring_', LiDAR_opts.properties.beam);

    elseif LiDAR_opts.mechanism.types(LiDAR_opts.mechanism.type) == "solid-state"
        %%% mechanics_noise_model (solid state sypes)
        R = rotx(0) * roty(0) * rotz(0);
        T = [0, 0, 0];
        pose = [R, T';zeros(1,3), 1];
        LiDAR_opts.pose.H = pose;
        LiDAR_opts.properties.mechanics_noise_model = opts.mechanics_noise_model; 
        LiDAR_opts.properties.array_distributions = ["Uniform-grid"];
        LiDAR_opts.properties.array_distribution = 1;
        LiDAR_opts.properties.diode_array = 20; 
        LiDAR_opts.properties.h_coverage = 80;  % -80 ~ 80
        LiDAR_opts.properties.v_coverage = 20;  % -80 ~ 80
        LiDAR_opts.properties.array_size = 2;
        LiDAR_opts.pose.range = 5;
        
        LiDAR_opts.properties.array_size = 40; % [m]
        LiDAR_opts.properties.array_emitters = 100; % how many emitters per side
        LiDAR_opts.properties.array_distributions = ["Uniform-grid"];
        LiDAR_opts.properties.array_distribution = 1;
        LiDAR_opts.properties.beam = 1; % 
    end


    %% Simulate environment
    disp("- Simulating LiDAR environment given provided obstacles...")
    [object_list, LiDAR_ring_points, all_points]= simulateLiDAR(object_list, boundary, LiDAR_opts);


    %% Plotting simulation
    disp("- Drawing simulated LiDAR environment...")
    % scatter3(fig_handle(3), all_points(1, :), all_points(2, :), all_points(3, :), 'b.')
    % cla(fig_handle(3))

    for beam_num = 1:LiDAR_opts.properties.beam
        scatter3(fig_handles(3), LiDAR_ring_points(beam_num).points.x, ...
                                 LiDAR_ring_points(beam_num).points.y, ...
                                 LiDAR_ring_points(beam_num).points.z, '.')
        hold(fig_handles(3), 'on')
    %     text(fig_handles(3), max(LiDAR_ring_points(beam_num).points.x), ...
    %                          max(LiDAR_ring_points(beam_num).points.y), ...
    %                          max(LiDAR_ring_points(beam_num).points.z), num2str(beam_num))
    end


    plotMultiplePolygonsVertices(fig_handles(3), object_list, color_list)
    plotOriginalAxis(fig_handles(3), LiDAR_opts.pose.H, 4, '-k')
    viewCurrentPlot(fig_handles(3), "LiDAR simulation (Scene " + num2str(scene) + ")")
%     set(fig_handles(3), 'visible', 'off')
%     set(fig_handles(3), 'Color', 'b')

    %% Plotting points on polygons
    % cla(fig_handle(4))
    disp("- Drawing points on obstacles...")
    plotMultiplePolygonsVertices(fig_handles(4), object_list, color_list)
    % scatter3(fig_handle(4), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')
    plotOriginalAxis(fig_handles(3), LiDAR_opts.pose.H, 1, '-k')
    for object = 1:length(object_list)
        scatter3(fig_handles(4), [object_list(object).ring_points.x], ...
                                 [object_list(object).ring_points.y], ...
                                 [object_list(object).ring_points.z], '.', 'MarkerEdgeColor',color_list{object})

        % Plot on separated plots
        % Noisy-points
        scatter3(fig_handles(4+object), [object_list(object).ring_points.x], ...
                                        [object_list(object).ring_points.y], ...
                                        [object_list(object).ring_points.z], '.', 'MarkerEdgeColor', color_list{object})
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
    viewCurrentPlot(fig_handles(4), "Rings on Objects (Scene " + num2str(scene) + ")", view_angle)
    view_angle = [90, 0];

    for object = 1:length(object_list)
        viewCurrentPlot(fig_handles(4+object), "Object 1 (Scene " + num2str(scene) + ")", view_angle)
    end

    if opts.show.simulator_statistics
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

        fprintf("\n\n------------------------\n")
        disp("- Numbers of points on each ring")
        disp("------------------------")
        for beam_num = 1:LiDAR_opts.properties.beam
            fprintf("\n--- ring %i\n", beam_num)
            fprintf("num_points of x: %i\n", length(LiDAR_ring_points(beam_num).points.x))
            fprintf("num_points of y: %i\n", length(LiDAR_ring_points(beam_num).points.y))
            fprintf("num_points of z: %i\n", length(LiDAR_ring_points(beam_num).points.z))
        end
    end
    ring_sorted = checkSimulatorRingOrder(LiDAR_ring_points);
    
    % save images
    if opts.save.images
        checkDirectory(opts.save_path);
        saveas(fig_handles(2),strcat(opts.save_path,'3DEnvironmentScene', num2str(scene),'.fig'));
        saveas(fig_handles(2),strcat(opts.save_path,'3DEnvironmentScene', num2str(scene),'.pdf'));
        saveas(fig_handles(3),strcat(opts.save_path,'LiDARSimulation', num2str(scene),'.fig')); 
        saveas(fig_handles(3),strcat(opts.save_path,'LiDARSimulation', num2str(scene),'.pdf'));
        saveas(fig_handles(4),strcat(opts.save_path,'objects', num2str(scene),'.fig'));
        saveas(fig_handles(4),strcat(opts.save_path,'objects', num2str(scene),'.pdf'));
    end
    
end
