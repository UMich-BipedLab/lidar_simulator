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

function tag = genFrameCoordinate(tag)
    % center and 4 vertices (top-left, bottom-left, bottom-right, top-right)
    tag.scatter3.x = zeros(1,5);
    tag.scatter3.y = [0, -tag.size/2, -tag.size/2,  tag.size/2, tag.size/2];
    tag.scatter3.z = [0,  tag.size/2, -tag.size/2, -tag.size/2, tag.size/2];
    
    % plot outer frame (top-left, bottom-left, bottom-right, top-right)
%     tag.plot3.x = zeros(1,8);
%     tag.plot3.y = [-tag.size/2, -tag.size/2, -tag.size/2, tag.size/2, tag.size/2, tag.size/2, tag.size/2, -tag.size/2];
%     tag.plot3.z = [tag.size/2, -tag.size/2, -tag.size/2, -tag.size/2, -tag.size/2, tag.size/2, tag.size/2, tag.size/2];

    % first 2 is vertical and horizontal
    % second 2 is two sides 
    tag.plot3.x = zeros(1, 2*(2*(tag.family + 2*(tag.black_border + tag.white_border) + 1))); 
    tag.plot3.y = [];
    tag.plot3.z = [];
    
    % Vertical lines
    corner = tag.family/2 + tag.black_border + tag.white_border;
    for i = -corner : corner
        tag.plot3.y = [tag.plot3.y, i*tag.grid_size, i*tag.grid_size];
        tag.plot3.z = [tag.plot3.z, corner * tag.grid_size, -corner*tag.grid_size];
    end
    
    % Horizontal lines
    for i = -corner : corner
        tag.plot3.y = [tag.plot3.y, corner * tag.grid_size, -corner*tag.grid_size];
        tag.plot3.z = [tag.plot3.z, i*tag.grid_size, i*tag.grid_size];
    end
end