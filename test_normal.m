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

clc,
target_size = 5;
objects2 = genShape("polygon", target_size, 4);
object2_mat = convertXYZstructToXYZmatrix(objects2);
object2_mat_h = converToHomogeneousCoord(object2_mat);
rpy = [0 30 0]; % in degree
xyz = [3 0 0];
moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
objects(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
objects(1).size = target_size;
[N, C] = computePlane(objects(1));
d = 1;
moved_point = movePointGivenAVector(C, -N, d);


num_objects = size(objects, 2);
fig_handles = createFigHandleWithNumber(1, 1, "test");
for i = 1:num_objects
    hold(fig_handles(1), 'on')
    points = [scan(i).diodes_array.diode.point];
    plotConnectedVerticesStructure(fig_handles(1), objects(i).object_vertices)
    scatter3(fig_handles(1), C(1), C(2), C(3))
    scatter3(fig_handles(1), moved_point(1), moved_point(2), moved_point(3))
end
plotOriginalAxis(fig_handles(1), eye(4), 1*0.5, '-k')
viewCurrentPlot(fig_handles(1), [],[-16,8])

disp("done")
