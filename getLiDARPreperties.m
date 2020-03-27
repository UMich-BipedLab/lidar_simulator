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

function properties = getLiDARPreperties(type, properties)
    if ~isfield(properties ,'rpm')
        properties.rpm = 300;
    end
    
    type_low = lower(type);
    switch type_low
        case lower('UltraPuckV2')
            properties = loadVelodyneUltraPuckV2(properties);
        case lower('Other')
            
        otherwise
            error("unknown LiDAR model: %s", type)
    end
    
    
    disp("--- LiDAR properties")
    fprintf("----- Name: %s\n", properties.name)
    fprintf("----- Beam: %i\n", properties.beam)
    fprintf("----- Range: %i [m]\n", properties.range)
    fprintf("----- RPM: %i\n", properties.rpm)
    fprintf("----- Azimuth resolution: %.2f [deg]\n", properties.az_resolution)
    fprintf("----- Mechanics noise model: %i \n", properties.mechanics_noise_model)
    fprintf("----- Sensor noise: [%.4f, %.4f, %.4f] [sigma]\n", properties.noise_sigma)
    fprintf("----- Return once: %i\n", properties.return_once)
end