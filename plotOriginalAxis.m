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

function plotOriginalAxis(fig_handle, pose_H, length, properteis)
    if ~exist('properteis', 'var')
        properteis = '-b';
    end
    if ~exist('length', 'var')
        length = 1;
    end
    origin = pose_H(:, 4);
    x_arrow = pose_H * ([length; 0; 0; 0]);
    y_arrow = pose_H * ([0; length; 0; 0]);
    z_arrow = pose_H * ([0; 0; length; 0]);
    
    quiver3(fig_handle, ...
            repelem(origin(1), 3)', repelem(origin(2), 3)', repelem(origin(3), 3)', ...
            [x_arrow(1) ; y_arrow(1) ;  z_arrow(1)], ...
            [x_arrow(2) ; y_arrow(2) ;  z_arrow(2)], ... 
            [x_arrow(3) ; y_arrow(3) ;  z_arrow(3)], ...
            properteis, 'fill', 'LineWidth', 3)
    text(x_arrow(1), y_arrow(1), z_arrow(1), "x")
    text(x_arrow(2), y_arrow(2), z_arrow(2), "y")
    text(x_arrow(3), y_arrow(3), z_arrow(3), "z")
end