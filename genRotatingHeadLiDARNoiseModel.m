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

function ring_noise_model = genRotatingHeadLiDARNoiseModel(ring_number, LiDAR_opts)
    names_of_noise_models = ["noSimpleMechanicalNoiseModel",... 
                             "whiteNoise",...
                             "simpleMechanicalNoiseModel", ...
                             "complexMechanicalNoiseModel", ...
                             "simpleHomogeneousNoiseModel", ...
                             "complexHomogeneousNoiseModel", ...
                             "simpleHomogeneousNoiseModelAddOnNoise"];
    if ring_number == 1
        fprintf("--- Using %s noise model \n", names_of_noise_models(LiDAR_opts.properties.mechanics_noise_model+1));
    end
    switch LiDAR_opts.properties.mechanics_noise_model
        case 0
            ring_noise_model = noSimpleMechanicalNoiseModel(ring_number, LiDAR_opts);
        case 1
            ring_noise_model = whiteNoise(ring_number, LiDAR_opts);
        case 2
            ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
                                                          LiDAR_opts);
        case 3
            ring_noise_model = complexMechanicalNoiseModel(ring_number, ...
                                                           LiDAR_opts);
        case 4
            ring_noise_model = simpleHomogeneousNoiseModel(ring_number, ...
                                                           LiDAR_opts);
        case 5
            ring_noise_model = complexHomogeneousNoiseModel(ring_number, ...
                                                            LiDAR_opts);
        case 6
            ring_noise_model = simpleHomogeneousNoiseModelAddOnNoise(ring_number, ...
                                                                     LiDAR_opts);
        otherwise
            error("No such LiDAR noise model (Mechanism model): %i", ...
                   LiDAR_opts.properties.mechanics_noise_model)
%             for i = 1:32
%                 ring_number = i
%             ring_noise_model = simpleMechanicalNoiseModel(ring_number, ...
%                                                           LiDAR_opts)
%             end
    end
            
    
end