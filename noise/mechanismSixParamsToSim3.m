function RT = mechanismSixParamsToSim3(rho, phi, delta_rho, delta_phi, delta_theta, s, h, v)
    % R: range
    % D: correction on R
    
    % eps: azimuth
    % beta: correcion on azimuth
    
    % delta: crrection elevation (raise up from x-y plane)

%     R2 = [sin(eps-beta) -cos(eps-beta) 0; cos(eps-beta) sin(eps-beta) 0; 0 0 1];
%     R3 = [cos(delta) 0 -sin(delta); 0 1 0; sin(delta) 0 cos(delta)];
%     T2 = [R+D; 0; 0];
%     RT = R2*R3*T2;
    delta_phi = deg2rad(delta_phi);
    phi = deg2rad(phi);
    delta_theta = deg2rad(delta_theta);
    
    T1 = sin(phi)*(delta_rho*cos(delta_theta)*cos(delta_phi)-h*sin(delta_phi)) - cos(phi)*(delta_rho*cos(delta_theta)*sin(delta_phi)+h*cos(delta_phi));
    T2 = sin(phi)*(delta_rho*cos(delta_theta)*sin(delta_phi)+h*cos(delta_phi)) + cos(phi)*(delta_rho*cos(delta_theta)*cos(delta_phi)-h*sin(delta_phi));
    RT = [s*cos(delta_theta)*cos(delta_phi) -s*cos(delta_theta)*sin(delta_phi) 0 T2;
          s*cos(delta_theta)*sin(delta_phi)  s*cos(delta_theta)*cos(delta_phi) 0 T1;
          0 0 (1/cos(delta_theta))^2 (s*rho+delta_rho)*sin(delta_theta)+v;
          0 0 0 1];
end