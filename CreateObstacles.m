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

function [object_list, color_list] = CreateObstacles(scene)
    fprintf("--- Using profile: %i\n", scene)
    switch scene
        case 0
            [object_list, color_list] = getScene0();
        case 1
            [object_list, color_list] = getScene1();
        case 2
            [object_list, color_list] = getScene2();
        case 3
            [object_list, color_list] = getScene3();
        case 4
            [object_list, color_list] = getScene4();
        case 5
            [object_list, color_list] = getScene5();
        case 6
            [object_list, color_list] = getScene6();
        case 7
            [object_list, color_list] = getScene7();
        case 8
            [object_list, color_list] = getScene8();
        case 9
            [object_list, color_list] = getScene9();
        case 10
            [object_list, color_list] = getScene10();
        case 11
            [object_list, color_list] = getScene11();
        case 12
            [object_list, color_list] = getScene12();
        case 13
            [object_list, color_list] = getScene13();
        case 14
            [object_list, color_list] = getScene14();
        case 21
            [object_list, color_list] = getScene21();
        case 22
            [object_list, color_list] = getScene22();
        case 23
            [object_list, color_list] = getScene23();
        case 24
            [object_list, color_list] = getScene24();
        case 25
            [object_list, color_list] = getScene25();
        case 26
            [object_list, color_list] = getScene26();
        case 27
            [object_list, color_list] = getScene27();
        case 28
            [object_list, color_list] = getScene28();
        case 29
            [object_list, color_list] = getScene29();
        case 30
            [object_list, color_list] = getScene30();
        case 91
            [object_list, color_list] = getScene91();
        case 92
            [object_list, color_list] = getScene92();
        otherwise
            error("inexistent scene: #%i", scene)
    end
    fprintf("--- This profile contains %i of target(s).\n", length(object_list))
    
    for object = 1:length(object_list)
        [object_list(object).normal, object_list(object).centroid] = computePlane(object_list(object));
    end
    
end