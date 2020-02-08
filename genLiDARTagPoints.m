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
% % Tag Family Setting
% tag.family = 4; % LiDARTag family
% tag.black_border = 1; 
% tag.white_border = 1;
% tag.grid_size = 0.5; % size of each grid
% tag.size = tag.grid_size * (tag.family + 2*tag.black_border + 2*tag.white_border);
% 
% tag.sim.type = ["Uniform", "Grid", "SimLiDAR"];
% tag.sim.distribution = 3; %
% tag.sim.noise_sigma = [0.1, 0.01, 0.1];
% tag.sim.num_points = 20^2;
% tag.sim.spacing = [0.1, 0.2, 0.3, 0.5];
% tag.sim.num_dense_list = [8, 6, 5, 3]; % number of beam around the center beam
% tag.sim.num_beams = 21; % how many beams on the tag
% tag.sim.num_dense = 3; % number of beam around the center beam
% tag.sim.center_beam = 11; % the beam at the center of the tag
% 
% 
% tag = t_genLiDARTagPoints(tag);
% tag = genFrameCoordinate(tag);
% figure(1)
% scatter3(tag.sim.noisy.points.x, tag.sim.noisy.points.y, tag.sim.noisy.points.z, '.r')
% hold on
% scatter3(tag.sim.noise_free.points.x, tag.sim.noise_free.points.y, tag.sim.noise_free.points.z, '.b')
% scatter3(tag.scatter3.x, tag.scatter3.y, tag.scatter3.z, 'ok')
% plot3(reshape(tag.plot3.x,2,[]),reshape(tag.plot3.y,2,[]), reshape(tag.plot3.z,2,[]), 'k')
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% hold off

function tag = genLiDARTagPoints(tag)
    method = tag.sim.type(tag.sim.distribution);
    switch method
        case 'Uniform'
            tag = genUniformPoints(tag);
        case {"Grid", "SimLiDAR"}
            tag = genGriddedPoints(tag);
    end
end
