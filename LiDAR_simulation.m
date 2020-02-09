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

% Scene number
scene = 6;

% Plotting parameters
num_handles = 8;
start_number = 1;
name = "testing";
fig_handles = createFigHandleWithNumber(num_handles, start_number, name);

%% Create objects
disp("- Generating obstacles...")
[object_list, color_list] = CreateObstacles(scene);

% Plot all polygons
plotMultiplePolygonsVertices(fig_handles(2), object_list, color_list)

% Workspace boundary
% boundary.x = [20, -20];
% boundary.y = [10, -10];
% boundary.z = [10, -10];
boundary.x = [20, -20];
boundary.y = [10, -10];
boundary.z = [10, -10];
boundary.vertices = createBoxVertices(boundary);
boundary.faces = createBoxFaces(boundary.vertices);
scatter3(fig_handles(2), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')

plotOriginalAxis(fig_handles(2), 1)
viewCurrentPlot(fig_handles(2), "3D environment (Scene " + num2str(scene) + ")")


%% LiDAR properties
disp("- Loading LiDAR properties...")
LiDAR_opts.properties.rpm = 600;
LiDAR_opts.properties.range = 50;
LiDAR_opts.centriod = [0 0 0];
LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
LiDAR_opts.ring_elevation = parseLiDARStruct(LiDAR_opts.properties.elevation_struct, 'ring_', LiDAR_opts.properties.beam);


%% Simulate environment
disp("- Simulating LiDAR environment given provided obstacles...")
[object_list, LiDAR_ring_points, all_points]= sim_LiDAR(object_list, boundary, LiDAR_opts);


%% Plotting simulation
disp("- Drawing simulated LiDAR environment...")
% scatter3(fig_handle(3), all_points(1, :), all_points(2, :), all_points(3, :), 'b.')
% cla(fig_handle(3))

for beam_num = 1:LiDAR_opts.properties.beam
    scatter3(fig_handles(3), LiDAR_ring_points(beam_num).points.x, ...
                             LiDAR_ring_points(beam_num).points.y, ...
                             LiDAR_ring_points(beam_num).points.z, '.')
    hold(fig_handles(3), 'on')
%     text(fig_handle(3), max(LiDAR_ring_points(beam_num).points.x), ...
%                         max(LiDAR_ring_points(beam_num).points.y), ...
%                         min(LiDAR_ring_points(beam_num).points.z), num2str(beam_num))
end


plotMultiplePolygonsVertices(fig_handles(3), object_list, color_list)
plotOriginalAxis(fig_handles(3), 1, '-k')
viewCurrentPlot(fig_handles(3), "LiDAR simulation (Scene " + num2str(scene) + ")")
set(fig_handles(3), 'visible', 'off')
set(fig_handles(3), 'Color', 'b')


%% Plotting points on polygons
% cla(fig_handle(4))
disp("- Drawing points on obstacles...")
plotMultiplePolygonsVertices(fig_handles(4), object_list, color_list)
% scatter3(fig_handle(4), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')
plotOriginalAxis(fig_handles(4), 1, '-k')
for object = 1:length(object_list)
    scatter3(fig_handles(4), [object_list(object).ring_points.x], ...
                             [object_list(object).ring_points.y], ...
                             [object_list(object).ring_points.z], color_list(object), '.')
    
    % Plot on separated plots
    scatter3(fig_handles(4+object), [object_list(object).ring_points.x], ...
                                    [object_list(object).ring_points.y], ...
                                    [object_list(object).ring_points.z], color_list(object), '.')
    plotConnectedVerticesStructure(fig_handles(4+object), object_list(object).object_vertices, 'b')
end
view_angle = [-86, 14];
viewCurrentPlot(fig_handles(4), "Rings on Objects (Scene " + num2str(scene) + ")", view_angle)
view_angle = [90, 0];
viewCurrentPlot(fig_handles(5), "Object 1 (Scene " + num2str(scene) + ")", view_angle)
viewCurrentPlot(fig_handles(6), "Object 2 (Scene " + num2str(scene) + ")", view_angle)
viewCurrentPlot(fig_handles(7), "Object 3 (Scene " + num2str(scene) + ")", view_angle)
viewCurrentPlot(fig_handles(8), "Object 4 (Scene " + num2str(scene) + ")", view_angle)

disp("------ Statistics ------")
disp("- std on each target")
disp("------------------------")
for object = 1:length(object_list)
    fprintf("\n--- Object %i\n", object)
    fprintf("std of x: %f\n", std([object_list(object).ring_points.x]))
    fprintf("std of y: %f\n", std([object_list(object).ring_points.y]))
    fprintf("std of z: %f\n", std([object_list(object).ring_points.z]))
end

% for beam_num = 1:LiDAR_opts.properties.beam
%     fprintf("\n--- ring %i\n", beam_num)
%     fprintf("num_points of x: %i\n", length(LiDAR_ring_points(beam_num).points.x))
%     fprintf("num_points of y: %i\n", length(LiDAR_ring_points(beam_num).points.y))
%     fprintf("num_points of z: %i\n", length(LiDAR_ring_points(beam_num).points.z))
% end

fprintf("\n\n\n")
disp("=================")
disp("Done All Process!")
disp("=================")