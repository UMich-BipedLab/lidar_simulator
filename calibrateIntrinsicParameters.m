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

function calibrateIntrinsicParameters(scene, opts)
    %% Simulate lidar environment
    [object_list, LiDAR_ring_points, ~, LiDAR_opts] = simuateLiDAREnvironment(scene, opts);


    %% Intrinsic Calibration
    opts.num_beams = LiDAR_opts.properties.beam;
    num_targets = length(object_list);
    [calib_param, plane, distance_original, distance, original_points, updated_points] = intrinsicCalibration(opts, object_list, num_targets);
    save(opts.save_path + "realExpDelta.mat", 'calib_param');

    %% show numerical results
    showCalibratedNumericalResults(distance, distance_original);

    % check if ring mis-ordered
     disp("Due to the parfar loop, numbers will no be in order")
    parfor object = 1:length(object_list)
        checkRingOrderWithOriginalPerTarget(original_points, updated_points, object, opts.num_beams)
    end

    %% Show calibrated rings
    if opts.plot.intrinsic_calibration        
        for object = 1:length(object_list)
            for ring = 1:LiDAR_opts.properties.beam
                if isempty( updated_points{object}(ring).points)
                    continue;
                end
                % draw ring in differnt color
                offset_color = max(1, mod(object+1, length(object_list)+1));
                scatter3(fig_handles(4+object), updated_points{object}(ring).points(1,:),...
                                     updated_points{object}(ring).points(2,:),...
                                     updated_points{object}(ring).points(3,:),...
                                     50, '.', 'MarkerEdgeColor', color_list{offset_color})
        %         scatter3(fig_handles(4+object), data_split_with_ring_cartesian{object}(ring).points(1,:),...
        %                              data_split_with_ring_cartesian{object}(ring).points(2,:),...
        %                              data_split_with_ring_cartesian{object}(ring).points(3,:),...
        %                              50, '.', 'MarkerEdgeColor', color_list{object})                   
                 text(fig_handles(4+object), mean(updated_points{object}(ring).points(1,:)), ...
                                             mean(updated_points{object}(ring).points(2,:)), ...
                                             mean(updated_points{object}(ring).points(3,:)), "C"+num2str(ring-1))
            end
        end
%         disp("Now plotting....")
%         plotCalibratedResults(num_targets, plane, updated_points, object_list);
%     %     plotCalibratedResults(num_targets, plane, data_split_with_ring_cartesian, data, opt_formulation(opts.method));
%         disp("Done plotting!")
    end

    fprintf("\n\n\n")
    disp("===========================")
    disp("Intrinsic Calibration Done!")
    disp("===========================")
end
