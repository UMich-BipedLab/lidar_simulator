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
% intrinsic_lib = '..\extrinsic_lidar_camera_calibration\';
% methods = ["\Lie\", "\BaseLine1\","\BaseLine3\"];

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


%% LiDAR simulator
scenes = [30];
% validaton_scene = [13,14];
noise_model = [2,3,6];
noise_model = [2]; % spinning lidar
scenes = [14];
% noise_model = [5];
opts.lidar.type = 1; %  ["rotating-head", "solid-state"]


%% Intrinsic calibration 
%%% Lie; BaseLine2; BaseLine2
opts.datatype = 2; % Experiment , Simulation
opts.iterative = 0; % for intrinsic calibration
opts.threshold = 0.5;

%% General options
opts.plot.intrinsic_calibration = 0;
opts.show.simulator_statistics = 0;
opts.show.intrinsic_statistics = 0;
opts.save.images = 0;
opts.save.calibration_result = 0;
opts.sensor_noise_enable = 0;


for model = 1:length(noise_model)
    opts.mechanics_noise_model = noise_model(model);
    for i = 1: size(scenes,2)
        scene = scenes(i);
        for method = 1:length(methods)
            opts.method = method;
            opts.save_path = save_path + num2str(opts.mechanics_noise_model)+ "/scene-" + num2str(scene) + "/" + methods(opts.method) + "/";
            checkDirectory(opts.save_path);
            calibrateIntrinsicParameters(scene, opts);
        end
    end
end