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

% Example:
% Determine the intersection of following the plane x+y+z+3=0 with the segment P0P1:
% The plane is represented by the normal vector n=[1 1 1]
% and an arbitrary point that lies on the plane, ex: V0=[1 1 -5]
% The segment is represented by the following two points
% P0=[-5 1 -1];
% P1=[1 2 3]   ;
% [I, check]=findIntersectionOfPlaneAndLine([1 1 1],[1 1 -5],[-5 1 -1],[1 2 3]);
% clc
% p_1=[-5 1 -1]';
% p_2=[1 2 3];
% vertices.x = [0 0 0 0 0 0];
% vertices.y = [0.4330 3.0616e-17 -0.4330 -0.4330 -5.3594e-16 0.4330];
% vertices.z = [0.2500 0.5000 0.2500 -0.2500 -0.5000 -0.2500];
% [I, check] = t_findIntersectionOfPlaneAndLine(vertices, p_1, p_2);

% function [I, check] = findIntersectionOfPlaneAndLine(n, V0, P0, P1)
function [I, check] = findIntersectionOfPlaneAndLine(polygon_vertices, p_1, p_2)
% plane_line_intersect computes the intersection of a plane and a segment(or
% a straight line)
% Inputs: 
%       n: normal vector of the Plane 
%       V0: any point that belongs to the Plane 
%       p_1: end point 1 of the segment P0P1
%       p_2:  end point 2 of the segment P0P1
%       clean_data: a flag to decide how to determine the plane
%
% Outputs:
%      I    is the point of interection 
%     Check is an indicator:
%      0 => disjoint (no intersection)
%      1 => the plane intersects p_1, p_2 in the unique point I
%      2 => the segment lies on the plane
%      3 => the intersection lies outside the segment p_1, p_2

    if isstruct(polygon_vertices)
        poly_mat = convertXYZstructToXYZmatrix(polygon_vertices);
    else
        poly_mat = polygon_vertices;
    end
    
    if ~isrow(p_1)
        p_1 = p_1';
    end
    
    if ~isrow(p_2)
        p_2 = p_2';
    end
        
    
    centroid = mean(poly_mat, 2)';
    [s, ~, ~] = svd(poly_mat - centroid');
    n = s(:, 3);
    n = n/norm(n);
    I=[0 0 0];
    
    u = p_2 - p_1;
    w = p_1 - centroid;
    D = dot(n, u);
    N = -dot(n, w);

    if abs(D) < 10^-7 % Parallel to plane
        if N == 0 % Lies on plane           
            check=2;
            return
        else
            % No intersection
            check=0;  
            return
        end
    end
    
    % Compute the intersection parameter
    sI = N / D;
    I = p_1+ sI.*u;
    if (sI < 0 || sI > 1)
        % The intersection point lies outside the segment, so there is no intersection
        check= 3;          
    else
        check=1;
    end
    if ~iscolumn(I)
        I = I';
    end
end