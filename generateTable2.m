addpath('..\extrinsic_lidar_camera_calibration\')
clc
v_scene = 13;
scenes = [8,9,10,11,12,6,7,14];
opts.datatype = "Simulation";
opts.show_statistics = 0;
noise_model = [2,3,6];
methods = ["\Lie\", "\BaseLine1\","\BaseLine3\"];
%%
% disp("- Generating obstacles...")
[object_list, color_list] = CreateObstacles(v_scene);
boundary.x = [40, -40];
boundary.y = [40, -40];
boundary.z = [40, -40];
boundary.vertices = createBoxVertices(boundary);
boundary.faces = createBoxFaces(boundary.vertices);

for model = 1:3
    opts.noise_model = noise_model(model);
    disp("- Loading LiDAR properties...")
    LiDAR_opts.properties.mechanics_noise_model = opts.noise_model; 
    LiDAR_opts.properties.sensor_noise_enable = 0;
    LiDAR_opts.properties.rpm = 1200; % 300, 60, 900, 1200
    LiDAR_opts.properties.range = 50;
    LiDAR_opts.properties.return_once = 0;
    LiDAR_opts.pose.centriod = [0 0 0];
    LiDAR_opts.pose.rpy = [0 0 0]; % deg (roll pitch yaw)
    LiDAR_opts.pose.H = constructHByRPYXYZ(LiDAR_opts.pose.rpy, LiDAR_opts.pose.centriod);
    LiDAR_opts.properties = getLiDARPreperties("UltraPuckV2", LiDAR_opts.properties);
    [LiDAR_opts.properties.ring_elevation, ...
     LiDAR_opts.properties.ordered_ring_elevation] = parseLiDARStruct(LiDAR_opts.properties, 'ring_', LiDAR_opts.properties.beam);
    
    disp("- Simulating LiDAR environment given provided obstacles...")
    [object_list, LiDAR_ring_points, all_points]= simulateLiDAR(object_list, boundary, LiDAR_opts);

    for i = 1: size(scenes,2)
        scene = scenes(i);
        for method = 3:3
            opts.method = method;
            opts.load_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\scene" + num2str(scene)+ methods(opts.method);
            opts.save_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\Validation13"+ "\scene" + num2str(scene)+ methods(opts.method);
            validation(scene, v_scene, opts, object_list, color_list, LiDAR_ring_points, LiDAR_opts)
        end
    end
end