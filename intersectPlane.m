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

function [P,L,check] = intersectPlane(N1,A1,N2,A2)
%plane_intersect computes the intersection of two planes(if any)
% Inputs: 
%       N1: normal vector to Plane 1
%       A1: any point that belongs to Plane 1
%       N2: normal vector to Plane 2
%       A2: any point that belongs to Plane 2
%
%Outputs:
%   P    is a point that lies on the interection straight line.
%   L    is the direction vector of the straight line
% check is an integer (0:Plane 1 and Plane 2 are parallel' 
%                      1:Plane 1 and Plane 2 coincide
%                      2:Plane 1 and Plane 2 intersect)

P=[0 0 0];
L=cross(N1,N2);
%  test if the two planes are parallel
if norm(L) < 10^-7                % Plane 1 and Plane 2 are near parallel
    V=A1-A2;
        if (dot(N1,V) == 0)         
            check=1;                    % Plane 1 and Plane 2 coincide
           return
        else 
            check=0;                   %Plane 1 and Plane 2 are disjoint
            return
        end
end
            
 check=2;
 
% Plane 1 and Plane 2 intersect in a line
%first determine max abs coordinate of cross product
[~, maxc] = max(abs(L));

%next, to get a point on the intersection line and
%zero the max coord, and solve for the other two
      
d1 = -dot(N1,A1);   %the constants in the Plane 1 equations
d2 = -dot(N2, A2);  %the constants in the Plane 2 equations
switch maxc
    case 1                   % intersect with x=0
        P(1)= 0;
        P(2) = (d2*N1(3) - d1*N2(3))/ L(1);
        P(3) = (d1*N2(2) - d2*N1(2))/ L(1);
    case 2                    %intersect with y=0
        P(1) = (d1*N2(3) - d2*N1(3))/ L(2);
        P(2) = 0;
        P(3) = (d2*N1(1) - d1*N2(1))/ L(2);
    case 3                    %intersect with z=0
        P(1) = (d2*N1(2) - d1*N2(2))/ L(3);
        P(2) = (d1*N2(1) - d2*N1(1))/ L(3);
        P(3) = 0;
end