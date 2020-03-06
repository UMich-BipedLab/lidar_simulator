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