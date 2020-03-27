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

format short g
scenes = [8,9,10,25,6,7];
noise_model = [2,3,6];
methods = ["\BaseLine1\", "\BaseLine3\","\Lie\"];

for model = 1:3
    summary = [];
    for i = 1: size(scenes,2)
        scene = scenes(i);
        block1 = [];
        block2 = [];
        col = [];
        for method = 1:size(methods,2)
            load_path = ".\results\noiseModel" + num2str(noise_model(model))+ "\Validation13"+ "\scene" + num2str(scene)+ methods(method) + "data.mat";
            load(load_path)
            block1 = [block1; results(1:16).mean_calibrated; results(1:16).mean_percentage];
            block2 = [block2; results(17:32).mean_calibrated; results(17:32).mean_percentage];
            mean_array = [results(:).mean_calibrated];
            nonzero_index = find(mean_array);
            nonzeros_mean = mean_array(nonzero_index);
            col = [col; mean(abs(nonzeros_mean))];
        end
        block1 = [results(1:16).mean_original; block1];
        block2 = [results(17:32).mean_original; block2];
        summary = [summary, col];
    end
    summary
end
