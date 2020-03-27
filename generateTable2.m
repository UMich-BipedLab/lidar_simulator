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

addpath('..\extrinsic_lidar_camera_calibration\')
clc
v_scene = 13;
scenes = [25];
opts.datatype = "Simulation";
opts.show_statistics = 1;
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
    [object_list, LiDAR_ring_points, all_points]= simulateRotatingHeadLiDAR(object_list, boundary, LiDAR_opts);

    for i = 1: size(scenes,2)
        scene = scenes(i);
        for method = 1:3
            opts.method = method;
            opts.load_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\scene" + num2str(scene)+ methods(opts.method);
            opts.save_path = ".\results\noiseModel" + num2str(opts.noise_model)+ "\Validation13"+ "\scene" + num2str(scene)+ methods(opts.method);
            validation(scene, v_scene, opts, object_list, color_list, LiDAR_ring_points, LiDAR_opts)
        end
    end
end