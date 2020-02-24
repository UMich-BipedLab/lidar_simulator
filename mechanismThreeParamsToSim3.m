function RT = mechanismThreeParamsToSim3(R, eps, D, beta, delta)
    % R: range
    % D: correction on R
    
    % eps: azimuth
    % beta: correcion on azimuth
    
    % delta: crrection elevation (raise up from x-y plane)

%     R2 = [sin(eps-beta) -cos(eps-beta) 0; cos(eps-beta) sin(eps-beta) 0; 0 0 1];
%     R3 = [cos(delta) 0 -sin(delta); 0 1 0; sin(delta) 0 cos(delta)];
%     T2 = [R+D; 0; 0];
%     RT = R2*R3*T2;
    s = 1;
    H = 0;
    V = 0;
    RT = mechanismSixParamsToSim3(R, eps, D, beta, delta, s, H, V);
end