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

function data = updateDataRaw(num_beams, num_targets, data_split_with_ring, delta, valid_rings_targets, method)
%     data = data_split_with_ring;
    data = cell(size(data_split_with_ring));
    for i =1:num_targets
        for ring = 1: num_beams
%             if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
%                 warning("Ring %i has been skipped in update function", ring)
            if (isempty(data_split_with_ring{i}(ring).points))
                  data{i}(ring).points= [];
                  continue;
            else
                if (method == "Lie")
%                     fprintf("target_num: %i, ring_num: %i\n", i, ring)
                    data{i}(ring).points =  delta(ring).Affine * data_split_with_ring{i}(ring).points;  
                elseif (method == "BaseLine1")
                    data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "Basic");
                elseif (method == "BaseLine2")
                    %Note: if a ring should be skipped, there won't be
                    %any corresponding delta parameter, thus we should
                    %directly transfer the point from spherical to
                    %cartesian coordinates
                    if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points,"Basic");
                    else
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "BaseLine2", delta(ring));
                    end
                    
                elseif (method == "BaseLine3")
                    if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "Basic");
                    else
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "BaseLine3", delta(ring));
                    end
                end            
            end
        end
    end
end