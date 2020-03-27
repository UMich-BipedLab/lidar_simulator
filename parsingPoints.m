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

function parsed_points_t = parsingPoints(points, object, num_grid_per_side)
    points = makeWideMatrix(points);
    num_points = size(points, 2);
    top_left = [max(object.object_vertices.x);
                max(object.object_vertices.y)
                max(object.object_vertices.z)];
    top_right = [max(object.object_vertices.x);
                 min(object.object_vertices.y)
                 max(object.object_vertices.z)];
    bottom_left = [max(object.object_vertices.x);
                   max(object.object_vertices.y)
                   min(object.object_vertices.z)];
    v1 = top_right - top_left;
    v1 = v1 / norm(v1);
    v2 = bottom_left - top_left;
    v2 = v2 / norm(v2);
    grid_length = object.size/num_grid_per_side;
    
    parsed_points_t(num_grid_per_side^2).points_mat = [];
    for i = 1:num_points
       v = points(1:3, i) - top_left;
       cell_column = ceil((dot(v, v1))/grid_length);
       cell_row = ceil((dot(v, v2))/grid_length);
       if cell_column<=0 || cell_row<=0 || cell_column>num_grid_per_side || cell_row>num_grid_per_side
           continue
       end
       index = cell_column + num_grid_per_side * (cell_row-1);
       parsed_points_t(index).points_mat = ...
           [parsed_points_t(index).points_mat, [points(1:3, i); 255; index]];
   end
end 