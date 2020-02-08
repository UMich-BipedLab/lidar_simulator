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

function rings = splitRegion(sim)
    num_spacing = length(sim.spacing);
    ring(num_spacing).beam_list = [];

    for i = 1:num_spacing
        if i == 1
            % lowest ring can't be negative
            lowest_ring = max(1, sim.center_beam - ceil(sim.num_dense_list(i)/2)); 

            % toppest ring can't be over maximum
            uppest_ring = min(sim.num_beams, sim.center_beam + floor(sim.num_dense_list(i)/2));
            rings(i).beam_list= linspace(lowest_ring, uppest_ring, uppest_ring-lowest_ring+1);
        else
            % lower rings
            lowest_ring_pre = lowest_ring - 1;
            lowest_ring = max(1, lowest_ring - floor(sim.num_dense_list(i)/2)); 
            rings(i).beam_list= linspace(lowest_ring, lowest_ring_pre, lowest_ring_pre-lowest_ring+1);
            
            % upper rings
            uppest_ring_pre = uppest_ring + 1;
            uppest_ring = min(sim.num_beams, uppest_ring + ceil(sim.num_dense_list(i)/2));
            rings(i).beam_list= [rings(i).beam_list, linspace(uppest_ring_pre, uppest_ring, uppest_ring-uppest_ring_pre+1)];
        end   
    end
end