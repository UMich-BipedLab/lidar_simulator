function RT = mechanismSixParamsToSim3(R, eps, D, beta, delta, s, H, V)
    % R: range
    % D: correction on R
    
    % eps: azimuth
    % beta: correcion on azimuth
    
    % delta: crrection elevation (raise up from x-y plane)

%     R2 = [sin(eps-beta) -cos(eps-beta) 0; cos(eps-beta) sin(eps-beta) 0; 0 0 1];
%     R3 = [cos(delta) 0 -sin(delta); 0 1 0; sin(delta) 0 cos(delta)];
%     T2 = [R+D; 0; 0];
%     RT = R2*R3*T2;
    beta = deg2rad(beta);
    eps = deg2rad(eps);
    delta = deg2rad(delta);
    
    
    T1 = sin(eps)*(D*cos(delta)*cos(beta)-H*sin(beta)) - cos(eps)*(D*cos(delta)*sin(beta)+H*cos(beta));
    T2 = sin(eps)*(D*cos(delta)*sin(beta)+H*cos(beta)) + cos(eps)*(D*cos(delta)*cos(beta)-H*sin(beta));
    RT = [s*cos(delta)*cos(beta) -s*cos(delta)*sin(beta) 0 T2;
          s*cos(delta)*sin(beta)  s*cos(delta)*cos(beta) 0 T1;
          0 0 (1/cos(delta))^2 (s*R+D)*sin(delta)+V;
          0 0 0 1];
end