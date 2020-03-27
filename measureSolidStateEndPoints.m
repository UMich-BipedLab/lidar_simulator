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


function new_points = measureSolidStateEndPoints(points, lidar_pose, desire_rotation, range)
    % desire_rotation in [roll pitch yaw] in degree
    if ~isvector(desire_rotation)
        error("rotation should be in [r p y] format")
    end
    
    [~, points] = checkHomogeneousCoord(points);

%     mean(points')
    
    x_G = points;
    H_GL = [lidar_pose(1:3,1:3)', -lidar_pose(1:3,1:3)*lidar_pose(1:3, 4);
           0 0 0 1];
    T_L =  constructHByRPYXYZ(desire_rotation, [range, 0 0]);
    H_LL_prime = [T_L(1:3,1:3), -T_L(1:3,1:3)*T_L(1:3, 4);
                  0 0 0 1];
    H_LG = inv(H_GL*H_LL_prime);
    new_points = H_LG * T_L * H_GL * x_G;
    
%     num_points = size(points,2);
%     new_points = ones(4, size(points, 2));
%     
%     for i = 1:num_points
%         x_G = points;
%         H_GL = [points_t.diode(:).T(1:3,1:3)', -points_t.diode(:).T(1:3,1:3)*points_t.diode(:).T(1:3,4);
%                0 0 0 1];
%         T_L =  constructHByRPYXYZ(desire_rotation, [range, 0 0]);
%         H_LG = inv(H_GL);
%         new_points(:, i) = H_LG * T_L * H_GL * x_G;
%     end
%     p = [0 0 0 1]
%     T = lidar_pose
%     (lidar_pose * p')'
%     H_GL
%     T_L
%     mean((H_GL * x_G)')
%     mean((T_L*H_GL * x_G)')
%     mean((H_LG*T_L*H_GL * x_G)')
%     vec = lidar_pose(1:3,1:3) * [1 0 0]';
%     movement = vec' .* [range, range, range];
%     H = constructHByRPYXYZ(desire_rotation, movement);
%     new_points = H * points;
end