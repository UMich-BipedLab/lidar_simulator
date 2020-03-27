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

% clc, clear
% tag.family = 4; % LiDARTag family
% tag.black_border = 1; 
% tag.white_border = 1;
% tag.grid_size = 0.5; % size of each grid
% tag.size = tag.grid_size * (tag.family + 2*tag.black_border + 2*tag.white_border);
% 
% tag.sim.num_points = 20^2;
% tag.sim.spacing = [0.1, 0.2, 0.5, 1];
% tag.sim.num_dense_list = [8, 6, 5, 3]; % number of beam around the center beam
% tag.sim.num_beams = 20; % how many beams on the tag
% tag.sim.center_beam = 9; % the beam at the center of the tag
% 
% n = ceil(sqrt(tag.sim.num_points));
% 
% [spaces_rings, spaced_rings_list] = t_genLiDARGrid(n, tag.size, tag.sim);
% figure(2)
% for i = 1:tag.sim.num_beams
%     scatter(spaces_rings(i).x(1,:), spaces_rings(i).y(1,:))
%     hold on
%     text(mean(spaces_rings(i).x(1,:),2), mean(spaces_rings(i).y(1,:)), num2str(i));
% end
% hold off

function [spaced_rings, spaced_rings_list] = genLiDARGrid(num_points, tag_length, sim) % y, z
    x_coord = linspace(0, tag_length, num_points);
    spaced_rings_list = splitRegion(sim);
%     disp('ring list')
%     spaced_rings_list.beam_list
    
    spaced_rings(sim.num_beams).x = zeros(1, num_points);
    spaced_rings(sim.num_beams).y = zeros(1, num_points);
    
    lower_length_original = 0;
    upper_length_original = 0;
    for i = 1:size(sim.spacing,2)
        % center ring
        for k = 1:num_points
            spaced_rings(sim.center_beam).x(1, k) = x_coord(k) - tag_length/2;
            spaced_rings(sim.center_beam).y(1, k) = 0;
        end
            
        if size(size([spaced_rings_list(i).beam_list], 2)) ~= 0
            current_spacing = sim.spacing(i);
            lower_list = find([spaced_rings_list(i).beam_list] < sim.center_beam);
            upper_list = find([spaced_rings_list(i).beam_list] > sim.center_beam);
            
            % lower list
            lower_length_original = lower_length_original + length(lower_list) * current_spacing;
            lower_length = lower_length_original;
            for j = 1:length(lower_list)
                current_beam = spaced_rings_list(i).beam_list(lower_list(j));
                for k = 1:num_points
                    spaced_rings(current_beam).x(1, k) = x_coord(k) - tag_length/2;
                    spaced_rings(current_beam).y(1, k) = -lower_length;
                end
                lower_length = lower_length - current_spacing;
            end
            
            % upper list
            upper_length = current_spacing + upper_length_original;
            for j = 1:length(upper_list)
                current_beam = spaced_rings_list(i).beam_list(upper_list(j));
                for k = 1:num_points
                    spaced_rings(current_beam).x(1, k) = x_coord(k) - tag_length/2;
                    spaced_rings(current_beam).y(1, k) = upper_length;
                end
                upper_length = upper_length + current_spacing;
            end
            upper_length_original = upper_length_original + length(upper_list) * current_spacing;
        end
    end
end