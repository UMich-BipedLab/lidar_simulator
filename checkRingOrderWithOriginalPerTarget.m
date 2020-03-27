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

function [ring_ordered, ring_misorder_list] = checkRingOrderWithOriginalPerTarget(data_split_with_ring_original, data_split_with_ring, target_num, num_ring)
    ring_order(num_ring) = struct();
    ring_order_original(num_ring) = struct();
    
    for ring = 1:num_ring
        ring_points = [];
        ring_points_org = [];
        if (isempty(data_split_with_ring_original{target_num}(ring).points))
            continue;
        else
            ring_points_org = [ring_points_org, data_split_with_ring_original{target_num}(ring).points];
        end

        if (isempty(data_split_with_ring{target_num}(ring).points))
            continue;
        else
            ring_points = [ring_points, data_split_with_ring{target_num}(ring).points];
        end
        
        ring_order(ring).ring = ring;
        if ~isempty(ring_points)
            ring_order(ring).ring_mean = mean(ring_points(1:3, :), 2)';
            ring_order(ring).z_axis = ring_order(ring).ring_mean(3);
        else
            ring_order(ring).ring_mean = [0 0 0];
            ring_order(ring).z_axis = 0;
        end
        
        ring_order_original(ring).ring = ring;
        if ~isempty(ring_points_org)
            ring_order_original(ring).ring_mean = mean(ring_points_org(1:3, :), 2)';
            ring_order_original(ring).z_axis = ring_order_original(ring).ring_mean(3);
        else
            ring_order_original(ring).ring_mean = [0 0 0];
            ring_order_original(ring).z_axis = 0;
        end
    end
    
    [~, idx] = sort([ring_order(:).z_axis]);
    ordered_ring = ring_order(idx);
    
    [~, idx] = sort([ring_order_original(:).z_axis]);
    ordered_ring_original = ring_order_original(idx);
    
    
    diff_vec = find([ordered_ring_original.ring] ~= [ordered_ring.ring]);
    
    if  isempty(diff_vec)
        fprintf("Rings on target #%i are ordered!\n", target_num)
        ring_misorder_list = ring_order;
    else
        warning("Rings on target #%i are mis-ordered.....\n", target_num)
        [ordered_ring_original.ring; ordered_ring.ring]'
    end
end