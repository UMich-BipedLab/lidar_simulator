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

function [delta, plane, valid_targets] = estimateIntrinsicLie(num_beams, num_targets, num_scans, data_split_with_ring, object_list, check_rings)
    delta(num_beams).H = struct();
    delta(num_beams).Affine = struct();
    if ~exist('check_rings', 'var')
        check_rings = 1;
    end
    
    for i = 1: num_scans
        % Calculate 'ground truth' points by projecting the angle onto the
        % normal plane
        %
        % Assumption: we have the normal plane at this step in the form:
        % plane_normal = [nx ny nz]

        % example normal lies directly on the x axis
        opt.corners.rpy_init = [45 2 3];
        opt.corners.T_init = [2, 0, 0];
        opt.corners.H_init = eye(4);
        opt.corners.method = "Constraint Customize"; %% will add more later on
        opt.corners.UseCentroid = 1;

        plane = cell(1,num_targets);
        
        for t = 1:num_targets
            if ~exist('object_list', 'var')
                X = [];
                for j = 1: num_beams
                    X = [X, data_split_with_ring{t}(j).points];
                end
                [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 1.5);
            else
                if ~isfield(object_list(t), 'centroid') || ~isfield(object_list(t), 'normal')
                    [normal, centroid] = computePlane(object_list(t));
                    plane{t}.centroid = [centroid; 1];
                    plane{t}.normals =  normal;
                    plane{t}.unit_normals = normal/(norm(normal));
                else
                    plane{t}.centroid =  [object_list(t).centroid; 1];
                    plane{t}.normals =  object_list(t).normal;
                    plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
                end
            end
        end


        opt.delta.rpy_init = [0 0 0];
        opt.delta.T_init = [0, 0, 0];
        opt.delta.scale_init = 1;
        opt.delta.H_init = eye(4);
        [delta, ~, valid_targets] = estimateDeltaLie(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets, check_rings);
    end
end