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


function [interior, status] = checkInsidePolygon(polygon_vertices, point)
    
    status = 0; % 0: outside of the polygon; 1: inside/on in the polygon;
    interior = [];
    if 1 %% Using angle sum method
        angle_sum = computeAngleSum(polygon_vertices, point); 
        if abs(angle_sum - 2*pi) < 1e-5
            status = 1;
            interior = point;
        end
    else
        %% Using projection
        if isstruct(polygon_vertices)
            poly_mat = convertXYZstructToXYZmatrix(polygon_vertices);
        else
            poly_mat = polygon_vertices;
        end

        if ~iscolumn(point)
            point = point';
        end

        centroid = mean(poly_mat, 2);
        [Uc, Vc, ~] = svd(poly_mat - centroid);
        Ind2D=[1,2];

    %     [Uc, ~] = fixSignsRotation(Uc, Vc);
    %     
    %     if abs(Uc(2,1)) > abs(Uc(3,1))
    %         Ind2D=[1,2];
    %     else
    %         Ind2D=[2,1];
    %     end

        poly_mat_2D = Uc' * (poly_mat - centroid);
        point_2D = Uc' * (point - centroid); % DO NOT need minus the centroid

        % TAKE ONLY Y-Z components
        poly_mat_2D = poly_mat_2D(Ind2D, :); % Project out the distance component
        point_2D = point_2D(Ind2D);


        convex_hull = convhull(poly_mat_2D(1, :),  poly_mat_2D(2, :));
        [in, on] = inpolygon(point_2D(1), point_2D(2), poly_mat_2D(1, convex_hull), poly_mat_2D(2, convex_hull));
        if in || on
            status = 1;
            interior = point;
        end
    end
end