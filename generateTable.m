clear, clc

addpath('..\extrinsic_lidar_camera_calibration\')

path = ".\results\";

scenes = [8,9,10,11,12,6,7,13,14];
% validation_scene = [13,14];
noise_model = [2,3,6];
methods = ["\Lie\", "\BaseLine1\","\BaseLine3\"];
addpath('..\extrinsic_lidar_camera_calibration\')

% Intrinsic calibration 
 % Lie; BaseLine2; BaseLine2
opts.datatype = "Simulation"; %Experiment , Simulation
opts.iterative = 0;
opts.show_results = 0;
opts.show_statistics = 0;

for model = 1:3
    opts.noise_model = noise_model(model);
    path = ".\results\" + "noiseModel" + num2str(opts.noise_model);
    for i = 1: size(scenes,2)
        scene = scenes(i);
        for method = 3:size(methods,2)
            opts.method = method;
            opts.save_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\scene" + num2str(scene)+ methods(opts.method);
            LiDAR_simulation(scene,opts);
        end
    end
end

