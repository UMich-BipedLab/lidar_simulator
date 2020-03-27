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

function segmentation_type = whiteNoiseSolidState(segmentation_type)
    %%% XXX Noise now only support when the array placed in x-axis
    segmentation_type.type = 'additive';
    if isfield(segmentation_type, 'angle')
        if isfield(segmentation_type.angle, 'point')
            segmentation_type.noisy_point = segmentation_type.angle.point;
            azimuth = segmentation_type.angle.azimuth;
            elevation = segmentation_type.angle.elevation;
            seed = abs(azimuth + elevation);
            
            %% x axis
            distance = norm(segmentation_type.angle.point);
            ratio = 0.02;
            noise_x = genRandomNumber(0, ratio*distance, seed, 1);
            
            %% y and z axis various by its angle
            ratio_y = 0.002;
            ratio_z = 0.002;

            noise_y = genRandomNumber(0, ratio_y * azimuth, seed, 1); % azimuth
            noise_z = genRandomNumber(0, ratio_z * elevation, seed, 1); % elevation
            noise = [noise_x noise_y noise_z];
        end
    else
        seed = 1;
        noise = genRandomNumber(0.01, 0.01, seed, 3);
    end
    
    segmentation_type.x = noise(1);
    segmentation_type.y = noise(2);
    segmentation_type.z = noise(3);
end