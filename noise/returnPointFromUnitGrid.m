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
% point = [0 2 3];
% division = 10;
% object.size = 5;

% t_returnTransformFromUnitGrid(point, division, object)

function segmentation_type = returnPointFromUnitGrid(segmentation_type)
    %%% XXX Noise now only support when the array placed in x-axis
    
    segmentation_type.type = 'subtraction';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            point = segmentation_type.angle.point;
            segmentation_type.noisy_point = point;
            
        end
    else
        error("This type is no yet supported")
    end
%     division = 4;
%     point_unit = point ./ segmentation_type.geom.objects.size;
%     [x, y, z] = sphere(division); 
%     x = x(division:end,:);
%     y = y(division:end,:);
%     z = z(division:end,:); 
%     z = z - z(1,1); % move to (x, y, 0)
%     r = 2;
%     a = gradient(z);
%     fig_handle = createFigHandle(1, "sphere");
%     hs = surf(fig_handle, r.*x,r.*y,r.*z, 'AlphaData',a, 'FaceAlpha','flat',...
%         'FaceColor','blue');
%     direction = [0 0 1]; 
%     rotate(hs, direction, 180/division) 
%     XData = hs.XData;
%     YData = hs.YData;
%     ZData = hs.ZData;
%     point_xy = [point_unit(3), point_unit(2), point_unit(1)];
%     for i = 1:division
%     %     j = mod(j, 4);
%         plane(i).object_vertices.x = [0 XData(1, i), XData(1, i+1)];
%         plane(i).object_vertices.y = [0 YData(1, i), YData(1, i+1)];
%         plane(i).object_vertices.z = [ZData(2,i) ZData(1, i), ZData(1, i+1)];
% 
%         plane(i).triangle.x = [0 XData(1, i), XData(1, i+1)];
%         plane(i).triangle.y = [0 YData(1, i), YData(1, i+1)];
%         plane(i).triangle.z = [0 ZData(1, i), ZData(1, i+1)];
%         [plane(i).normal, plane(i).centroid, plane(i).R] = computePlaneReturnR(plane(i));
%     %     scatter3(plane(i).triangle.x, plane(i).triangle.y, plane(i).triangle.z, 'y', 'fill')
%         plane(i).angle_sum = computeAngleSum(plane(i).triangle, point_xy); 
%         plane(i).status = 0;
%         if abs(plane(i).angle_sum - 2*pi) < 1e-5
%             plane(i).status = 1;
%             interior = point;
%         end
%     end
%     [select_angle, which_polygon] = min(abs([plane(:).angle_sum]- 2*pi));
%     [new_point, ~, intersect] = findIntersectionOfPlaneAndLine(plane(which_polygon), ...
%                                                                 point_xy(1:3), point_xy + [0 0 1]);
    projected_point = projectPointToSphere(point, segmentation_type.geom.objects.size);
    segmentation_type.noisy_point =  segmentation_type.geom.objects.size * [projected_point(3); projected_point(2); projected_point(1)];
    segmentation_type.x = projected_point(3)-0.1;
    segmentation_type.y = projected_point(2);
    segmentation_type.z = projected_point(1);
end