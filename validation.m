%validation
%General parameters
clear; clc

t_scene = 8; %training scene
v_scene = 13; %validation scene
show_statistics = 1;
addpath('..\extrinsic_lidar_camera_calibration\')

filename = strcat('.\results\parameter',num2str(t_scene),'.mat');
load(filename);
fprintf("Calibration parameter from scene %i is loaded! \n", t_scene);

% Create objects
disp("- Generating obstacles...")
[object_list, color_list] = CreateObstacles(v_scene);


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

plotOriginalAxis(fig_handles(2), 1)
viewCurrentPlot(fig_handles(2), "3D environment (Scene " + num2str(v_scene) + ")")

%% LiDAR properties
disp("- Loading LiDAR properties...")
LiDAR_opts.noise_model = 1; % 1: simpleMechanicalNoiseModel
LiDAR_opts.properties.rpm = 1200; % 300, 60, 900, 1200
LiDAR_opts.properties.range = 50;
LiDAR_opts.properties.noise_enable = 0;
LiDAR_opts.centriod = [0 0 0];
LiDAR_opts.return_once = 0;
LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
[LiDAR_opts.properties.ring_elevation, ...
LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties.elevation_struct, 'ring_', LiDAR_opts.properties.beam);


%% Simulate environment
disp("- Simulating LiDAR environment given provided obstacles...")
[object_list, LiDAR_ring_points, all_points]= simulateLiDAR(object_list, boundary, LiDAR_opts);

%% Calibrate point clouds
disp("~ Calibrating LiDAR Point clouds")
[object_list, LiDAR_ring_points] = simulateCalibratedLiDAR(object_list,LiDAR_ring_points,LiDAR_opts, delta);

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


plotMultiplePolygonsVertices(fig_handles(3), object_list, color_list)
plotOriginalAxis(fig_handles(3), 1, '-k')
viewCurrentPlot(fig_handles(3), "LiDAR simulation (Scene " + num2str(v_scene) + ")")
set(fig_handles(3), 'visible', 'off')
set(fig_handles(3), 'Color', 'b')

% Plotting points on polygons
% cla(fig_handle(4))
disp("- Drawing points on obstacles...")
plotMultiplePolygonsVertices(fig_handles(4), object_list, color_list)
% scatter3(fig_handle(4), [boundary.vertices.x], [boundary.vertices.y], [boundary.vertices.z], 'fill')
plotOriginalAxis(fig_handles(4), 1, '-k')
for object = 1:length(object_list)
    scatter3(fig_handles(4), [object_list(object).ring_points.x], ...
                             [object_list(object).ring_points.y], ...
                             [object_list(object).ring_points.z], '.', 'MarkerFaceColor',color_list{object})
    hold(fig_handles(4), 'on')
    scatter3(fig_handles(4), [object_list(object).calibrated_ring_points.x], ...
                         [object_list(object).calibrated_ring_points.y], ...
                         [object_list(object).calibrated_ring_points.z], 'x', 'MarkerFaceColor',color_list{object})
    % Plot on separated plots
    % Noisy-points
    scatter3(fig_handles(4+object), [object_list(object).ring_points.x], ...
                                    [object_list(object).ring_points.y], ...
                                    [object_list(object).ring_points.z], '.', 'MarkerFaceColor', color_list{object})
    hold(fig_handles(4+object), 'on')
    scatter3(fig_handles(4+object), [object_list(object).calibrated_ring_points.x], ...
                                    [object_list(object).calibrated_ring_points.y], ...
                                    [object_list(object).calibrated_ring_points.z], 'x', 'MarkerFaceColor', color_list{object})   
    
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
end

if show_statistics
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
    data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(object_list(t).points_mat, LiDAR_opts.properties.beam);
    calibrated_data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(object_list(t).calibrated_points_mat, LiDAR_opts.properties.beam);
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
