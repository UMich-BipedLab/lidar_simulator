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

function properties = loadVelodyneUltraPuckV2(properties)
    %% Basics
    properties.name = "UltraPuckV2";    
    properties.beam = 32;
    
    %% Noise
    % std(BagData(current_index).lidar_target(1).scan.pc_points')
    % large tag: std 0.0130    0.0444    0.0457
    % small tag: std 0.0797    0.2187    0.1724
    properties.noise_sigma = [0.05    0.008   0.005];
    %             properties.noise_sigma = [2*sqrt(0.05), sqrt(0.05)/2, 0.01]; % 
    %             properties.noise_sigma = [0.1, 0.1, 0.1]; % under 50m
    %             properties.noise_sigma = [0.2, 0.2, 0.2]; % 50m to 200m
    %             properties.noise_sigma = [0, 0, 0];
    
    %% Azimuth resolution
    switch properties.rpm 
        case 300
            properties.az_resolution = 0.1; % in degree
        case 600
            properties.az_resolution = 0.2; % in degree
        case 900
            properties.az_resolution = 0.3; % in degree
        case 1200
            properties.az_resolution = 0.4; % in degree
        otherwise
            properties.az_resolution = 0.1; % in degree
    end

    %% Elevation angles by rings
    properties.elevation_struct = struct(...
                                        'ring_0',  -25.0, ...   % 0
                                        'ring_3',  -15.639, ... % 3
                                        'ring_4',  -11.310, ... % 4
                                        'ring_7',  -8.843, ...  % 7
                                        'ring_8',  -7.254, ...  % 8
                                        'ring_11', -6.148, ...  % 11
                                        'ring_12', -5.333, ...  % 12
                                        'ring_16', -4.667, ...  % 16
                                        'ring_15', -4.0, ...    % 15
                                        'ring_19', -3.667, ...  % 19
                                        'ring_20', -3.333, ...  % 20
                                        'ring_24', -3.0, ...    % 24
                                        'ring_23', -2.667, ...  % 23
                                        'ring_27', -2.333, ...  % 27
                                        'ring_28', -2.0, ...    % 28
                                        'ring_2',  -1.667, ...  % 2
                                        'ring_31', -1.333, ...  % 31
                                        'ring_1',  -1.0, ...    % 1
                                        'ring_6',  -0.667, ...  % 6
                                        'ring_10', -0.333, ...  % 10
                                        'ring_5',   0.0, ...    % 5
                                        'ring_9',   0.333, ...  % 9
                                        'ring_14',  0.667, ...  % 14
                                        'ring_18',  1.0, ...    % 18
                                        'ring_13',  1.333, ...  % 13
                                        'ring_17',  1.667, ...  % 17
                                        'ring_22',  2.333, ...  % 22
                                        'ring_21',  3.333, ...  % 21
                                        'ring_26',  4.667, ...  % 26
                                        'ring_25',  7.0, ...    % 25
                                        'ring_30', 10.333, ...  % 30
                                        'ring_29', 15.0 ...     % 29
                                        );
end