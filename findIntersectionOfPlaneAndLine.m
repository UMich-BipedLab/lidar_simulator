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
% clc
% p_1=[0
%      0.5000
%      0.5000];
% p_2=[5
%      0.5000
%      0.5000];
% objects2 = genShape("polygon", 1, 4);
% object2_mat = convertXYZstructToXYZmatrix(objects2);
% object2_mat_h = converToHomogeneousCoord(object2_mat);
% rpy = [0 0 0]; % in degree
% xyz = [2 0 0];
% moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
% objects.object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
% [I, normal, check] = t_findIntersectionOfPlaneAndLine(objects, p_1, p_2)

% function [I, check] = findIntersectionOfPlaneAndLine(n, V0, P0, P1)
function [I, normal, check] = findIntersectionOfPlaneAndLine(object, p_1, p_2)
% plane_line_intersect computes the intersection of a plane and a segment(or
% a straight line)
% Inputs: 
%       p_1: end point 1 of the segment p_1, p_2
%       p_2: end point 2 of the segment p_1, p_2
%
% Outputs:
%      n: normal vector of the Plane 
%      I    is the point of interection 
%     Check is an indicator:
%      0 => disjoint (no intersection)
%      1 => the plane intersects p_1, p_2 in the unique point I
%      2 => the segment lies on the plane
%      3 => the intersection lies outside the segment p_1, p_2

    p_1 = makeRow(p_1);
    p_2 = makeRow(p_2);

    % Convert to matrix
    if isstruct(object.object_vertices)
        poly_mat = convertXYZstructToXYZmatrix(object.object_vertices);
    else
        poly_mat = object.object_vertices;
    end
    
    if ~isfield(object, 'normal') ||  ~isfield(object, 'cntroid')
        [normal, centroid] = computePlane(poly_mat);
    end

    [I, check] = findIntersectionOfPlaneAndLineGivenPlane(normal, centroid, p_1, p_2);
end