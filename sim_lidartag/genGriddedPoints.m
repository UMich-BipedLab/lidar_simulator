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

function tag = genGriddedPoints(tag)
    % contruct grid on the target frame
    %             n = ceil(sqrt(app.num_points_target));
    %             y = linspace(0, app.target_len_, n);
    %             z = linspace(0, app.target_len_, n);
    %             app.noise_free_points_ = zeros(4, n^2); % x
    %
    %             for i = 1:n
    %                 for j = 1:n
    %                     app.noise_free_points_(2,(i-1)*n + j) = y(j) - app.target_len_/2;
    %                     app.noise_free_points_(3,(i-1)*n + j) = z(i) - app.target_len_/2;
    %                 end
    %             end
    n = ceil(sqrt(tag.sim.num_points));
    if tag.sim.type(tag.sim.distribution) == "Grid"
        [y, z] = genUniformGrid(n, tag.size);
    elseif tag.sim.type(tag.sim.distribution) == "SimLiDAR"
        [tag.LiDARTag.spaced_rings, tag.LiDARTag.spaced_rings_list] = genLiDARGrid(n, tag.size, tag.sim);
%         figure(2)
        y = [];
        z = [];
        for i = 1:tag.sim.num_beams
            y = [y, tag.LiDARTag.spaced_rings(i).x(1,:)];
            z = [z, tag.LiDARTag.spaced_rings(i).y(1,:)];
%             hold on
%             text(mean(spaces_rings(i).x(1,:),2), mean(spaces_rings(i).y(1,:)), num2str(i));
        end
%         scatter(y, z, 'r.')
%         hold off
    end
    tag.sim.noise_free.points.x = zeros(1, size(y,2)); % x
    tag.sim.noise_free.points.y = y;
    tag.sim.noise_free.points.z = z;

    % mimic LiDAR's performance
    rand_z = getNoise(0, tag.sim.noise_sigma(3), size(y,2)); % ring
    rand_y = getNoise(0, tag.sim.noise_sigma(2), size(y,2)); % noise on ring
    rand_x = getNoise(0, tag.sim.noise_sigma(1), size(y,2)); % noise on depth

    % noisy data
    tag.sim.noisy.points.z = tag.sim.noise_free.points.z + rand_z;
    tag.sim.noisy.points.y = tag.sim.noise_free.points.y + rand_y;
    tag.sim.noisy.points.x = tag.sim.noise_free.points.x + rand_x;
end