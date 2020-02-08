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

% clc
% vec1 = [0 -1 0];
% vec2 = [0 0 -1];
% corner = [0 -4 -4];
% img = imread('tag16_05_00001.png'); % Load LiDARTag
% peak = 1;
% num_bit = 8;
% grid_size = 1;
% points.x = zeros(1, num_bit^2);
% points.y = repmat(linspace(-4, num_bit/2, num_bit), 1, num_bit);
% points.z = repelem(linspace(-4, num_bit/2, num_bit), num_bit);
% figure(3);
% hold off
% points = t_genIntensityBasedOnImageTag(corner, vec1, vec2, img, peak, num_bit, grid_size, points, 0);
% 
% figure(3)
% for i = 1:size(points.x, 2)
%     scatter3(points.x(i), points.y(i), points.z(i), 1000, 'or', 'MarkerFaceColor', 'r', 'MarkerFaceAlpha', points.normalized_intensity(i))
%     hold on
% end
% hold on
% grid on
% axis equal
% xlabel('x') 
% ylabel('y')
% zlabel('z')
% hold off

function points = genPointIntensityBasedOnImageTag(corner, vec1, vec2, tag_img, g_peak, num_bit, grid_size, points, binary)
    vec1 = vec1 ./ norm(vec1);
    vec2 = vec2 ./ norm(vec2);
%     figure(3)
    for i = 1:size(points.x, 2)
        point = [points.x(i) points.y(i) points.z(i)];
%         scatter3(points.x(i), points.y(i), points.z(i))
        vec_p = point - corner;
        [index_y, grid_center_y, len_from_center_y] = getTagIndex(vec_p, vec1, corner(2), grid_size, num_bit);
        [index_z, grid_center_z, len_from_center_z] = getTagIndex(vec_p, vec2, corner(3), grid_size, num_bit);

        test = 10;
        sigma = [grid_size/test, 0; 0, grid_size/test];
        intensity = mvnpdf(point(2:3), [grid_center_y, grid_center_z], sigma);
        if tag_img(index_z, index_y, 1)  > 128
            % white
            sign = g_peak;
            if binary
                sign = 1;
            end
        else
            % black
            sign = -g_peak;
            if binary
                sign = 0;
            end
        end
        points.intensity(1, i) = sign * intensity;
%         sign * intensity
        if binary
%             [index_y, index_z]
%             [len_from_center_y, len_from_center_z]
%             tag_img(index_y, index_x, 1)
%             sign * 1
%             scatter3(0, grid_center_y, grid_center_z, 'ok');
%             hold on
            points.intensity(1, i) = sign * 1;
%             scatter3(points.x(i), points.y(i), points.z(i), 1000, 'or');
        end      
    end
%     points.normalized_intensity = 255 * (points.intensity - min(points.intensity)) / (max(points.intensity) - min(points.intensity));
    points.normalized_intensity = rescale(points.intensity, 0, 255); 
    points.normalized_intensityM1P1 = rescale(points.intensity, -1, 1); 
    % white to black, black to white only for visualization
    if ~binary
        points.plotting.normalized_intensity = rescale(-points.intensity, 0, 1);
    else
        points.plotting.normalized_intensity = points.intensity;
    end
%     scatter3(points.x(i), points.y(i), points.z(i), 1000, 'or', 'MarkerFaceColor', 'r', 'MarkerFaceAlpha', points.normalized_intensity(i));
end