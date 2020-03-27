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

function noisy_point = addTwoTypesOfNoise(point, sensor_noise, noisy_model)
    if isempty(sensor_noise)
        sensor_noise.rand_x = 0;
        sensor_noise.rand_y = 0;
        sensor_noise.rand_z = 0;
    end
    if noisy_model.type == "additive"
        noisy_point = point(1:3) + ...
                       [sensor_noise.rand_x + noisy_model.x; ...
                        sensor_noise.rand_y + noisy_model.y; ...
                        sensor_noise.rand_z + noisy_model.z;];
    elseif noisy_model.type =="subtraction"
        noisy_point = point(1:3) + ...
                       [sensor_noise.rand_x - noisy_model.x; ...
                        sensor_noise.rand_y - noisy_model.y; ...
                        sensor_noise.rand_z - noisy_model.z;];
    elseif noisy_model.type == "transformation"
        [~, point_h] = checkHomogeneousCoord(point(1:3));
        if isfield(noisy_model, 'sim3')
            noisy_point = noisy_model.sim3 * point_h;
            noisy_point = noisy_point(1:3)  + ...
                           [sensor_noise.rand_x; ...
                            sensor_noise.rand_y; ...
                            sensor_noise.rand_z;];
        else
            error("No sim3 in the struct")
        end
    elseif noisy_model.type == "noisy_point"
        noisy_point = noisy_model.noisy_point;
    else
        error("No such noise type: %s, model_noise.type")
    end
end