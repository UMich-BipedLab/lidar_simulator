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
% boundary.vertices.x = [20 20 20 20 -20 -20 -20 -20];
% boundary.vertices.y = [10 10 -10 -10 10 10 -10 -10];
% boundary.vertices.z = [10 -10 10 -10 10 -10 10 -10];
% boundary.faces = createBoxFaces(boundary.vertices);
% point = [25, 25, 5]';
% LiDAR_centroid = [0 0 0];
% point = t_limitInBoundaryWithBoundaryPlanes(point, LiDAR_centroid, boundary)


function point = limitInBoundaryWithBoundaryPlanes(p_1, p_2, boundary)
    % p_1 is the test point
    % p_2 is the origin (LiDAR_center)
    if ~iscolumn(p_1)
        p_1 = p_1';
    end
    
    flag_intersect = 0;
    intersect(size(boundary.faces, 2)) = struct();
    for i = 1:size(boundary.faces, 2)
        intersect(i).d = norm(p_1 - p_2);
%         intersect(i).point = p_1;
        [I, check] = findIntersectionOfPlaneAndLineGivenPlane(boundary.faces(i).normal, ...
                          boundary.faces(i).centroid, p_1, p_2);
        intersect(i).point = I;
        if check == 1 % intersect with the plane 
            flag_intersect = 1;
            intersect(i).point = I;
            intersect(i).d = norm(I - p_2);
        end
    end
    
    if flag_intersect
        [~, index] = min([intersect(:).d]);
        point = intersect(index).point;
    else
        point = p_1;
    end
    
    if isempty(point)
        dbstop in limitInBoundaryWithBoundaryPlanes at 65
        error("point is empty")
    end
    
    
%     dbstop if isempty(point)
%     disp("---")
%     index
%     intersect(:).d
%     intersect(:).point
%     flag_intersect
%     point
end