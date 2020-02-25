function XYZ = complexMechanicalNoiseModelToXYZ(R, eps, D, beta, delta, s, H, V)
rho = s*R + D;
XYZ=[rho*cos(delta)*( sin(eps)*cos(beta)-cos(eps)*sin(beta) ) - H*( cos(eps)*cos(beta) + sin(eps)*sin(beta) );
     rho*cos(delta)*( cos(eps)*cos(beta)+sin(eps)*sin(beta) ) + H*( sin(eps)*cos(beta) - cos(eps)*sin(beta) );
     rho*sin(delta) + V;
     1];
end