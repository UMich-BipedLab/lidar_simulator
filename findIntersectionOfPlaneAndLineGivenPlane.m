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


function [intersection_point, check] = findIntersectionOfPlaneAndLineGivenPlane(normal, centroid, p_1, p_2)
    if ~isrow(p_1)
        p_1 = p_1';
    end
    
    if ~isrow(p_2)
        p_2 = p_2';
    end
    
    if ~isrow(centroid)
        centroid = centroid';
    end
    
    intersection_point = [];
    u = p_2 - p_1;
    w = p_1 - centroid;
    D = dot(normal, u);
    N = -dot(normal, w);
    
    %% Check if two end points lie on the plane
%     if dot(p_1 - centroid, normal) < 1e-5
%         check = 1;
%         intersection_point = makeColumn(p_1);
%         disp("p1 intersect")
%         
%         return
%     elseif dot(p_2 - centroid, normal) < 1e-5
%         check = 1;
%         intersection_point = makeColumn(p_2);
%         disp("p2 intersect")
%         return
%     end
    
    %% Check if the segment intersect with the plane


    if abs(D) < 10^-7 % Parallel to plane
        if N == 0 % Lies on plane           
            check = 2;
            return
        else
            % No intersection
            check=0;  
            return
        end
    end
    
    % Compute the intersection parameter
    sI = N / D;
    intersection_point = p_1+ sI.*u;
    if (sI < 0 || sI > 1)
        % The intersection point lies outside the segment, so there is no intersection
        check= 3;          
    else
        check=1;
    end
    intersection_point = makeColumn(intersection_point);
end