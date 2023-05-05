function [trajd] = Flat2State(t, sys_prms, ref_prms)
% compute the desired state based on higher-order 
% derivative of the flat outputs
% the basis for the flat outputs is given in 

% compute the desired reference 
% yaw angle reference 
d2psi = ref_prms.psi_a2;
dpsi = ref_prms.psi_a2 * t + ref_prms.psi_a1;
psi = ref_prms.psi_a2 * t^2/2 + ref_prms.psi_a1 * t + ref_prms.psi_a0; 

% position reference
omega_square = ref_prms.omega.^2;
phi = ref_prms.omega * t + ref_prms.phi0;
cphi = ref_prms.A .* cos(phi);
sphi = ref_prms.A .* sin(phi);
pos = cphi + ref_prms.pos_a0 + ref_prms.pos_a1 * t...
       + ref_prms.pos_a2 * t^2/2;
vel = -ref_prms.omega .* sphi + ref_prms.pos_a1 + ref_prms.pos_a2 * t;
acc = - omega_square .* cphi + ref_prms.pos_a2;
dacc = ref_prms.omega .*omega_square .* sphi; 
d2acc = omega_square.^2 .* cphi;


% get the desired rotation matrix 
% using the symbolic operation
% ---------------------------------------------------------------------
% out1 = Deriv2Rot(psi, dpsi, d2psi, acc(1), dacc(1), d2acc(1),...
%                                                        acc(2), dacc(2), d2acc(2),...
%                                                        acc(3), dacc(3), d2acc(3), sys_prms.g);
%                                                    
% R = reshape(out1(:, 1), 3, 3);
% dR = reshape(out1(:, 2), 3, 3);
% d2R = reshape(out1(:, 3), 3, 3);
% % compute the omega
% Omega_hat = R\dR;
% dOmega_hat = R\d2R - Omega_hat * Omega_hat;
% ---------------------------------------------------------------------
% compute using differential flatness
    b1d = [cos(psi); sin(psi); 0];
    db1d = dpsi *[-sin(psi); cos(psi); 0];
    d2b1d = -dpsi^2 * b1d + d2psi * [-sin(psi); cos(psi);0];

    fb3 = sys_prms.m * (acc + [0; 0; sys_prms.g]);
    norm_fb3 = norm(fb3) ;
    f = norm_fb3 ;
    b3 = fb3 / norm_fb3 ;
    b3_b1d = cross(b3, b1d) ;
    norm_b3_b1d = norm(b3_b1d) ;
    b1 = - cross(b3, b3_b1d) / norm_b3_b1d ;
    b2 = cross(b3, b1) ;
    R = [b1 b2 b3] ;
    
    dfb3 = sys_prms.m * dacc;
    dnorm_fb3 = dot(fb3, dfb3) / norm_fb3 ;
    db3 = (dfb3*norm_fb3 - fb3*dnorm_fb3) / norm_fb3^2 ;
    db3_b1d = cross(db3, b1d) + cross(b3, db1d) ;
    dnorm_b3_b1d = dot(b3_b1d, db3_b1d) / norm_b3_b1d ;
    db1 = (-cross(db3,b3_b1d)-cross(b3,db3_b1d) - b1*dnorm_b3_b1d) /norm_b3_b1d ;
    db2 = cross(db3, b1) + cross(b3, db1) ;
    dR = [db1 db2 db3] ;
    Omega_hat = R' * dR;
    Omega = vee(Omega_hat) ;
    
    d2fb3 = sys_prms.m*d2acc;
    d2norm_fb3 = (dot(dfb3, dfb3)+dot(fb3, d2fb3) - dnorm_fb3*dnorm_fb3)/norm_fb3 ;
    d2b3 = ( (d2fb3*norm_fb3+dfb3*dnorm_fb3 - dfb3*dnorm_fb3-fb3*d2norm_fb3)*norm_fb3^2- db3*norm_fb3^2*2*norm_fb3*dnorm_fb3) / norm_fb3^4 ;
    d2b3_b1d = cross(d2b3, b1d)+cross(db3, db1d) + cross(db3, db1d)+cross(b3, d2b1d) ;
    d2norm_b3_b1d = ( (dot(db3_b1d,db3_b1d)+dot(b3_b1d,d2b3_b1d))*norm_b3_b1d - dot(b3_b1d, db3_b1d)*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
    d2b1 = ( (-cross(d2b3,b3_b1d)-cross(db3,db3_b1d) - cross(db3,db3_b1d)-cross(b3,d2b3_b1d) - db1*dnorm_b3_b1d-b1*d2norm_b3_b1d )*norm_b3_b1d - db1*norm_b3_b1d*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
    d2b2 = cross(d2b3, b1)+cross(db3, db1) + cross(db3, db1)+cross(b3, d2b1) ;
    d2R = [d2b1 d2b2 d2b3] ;
    dOmega = vee( dR' * dR + R'*d2R ) ; %vee( dR'*dR + R'*d2R, true ) ;
    
    M = sys_prms.J*dOmega + cross(Omega, sys_prms.J*Omega) ;


trajd.R = R;
trajd.dR = dR;
trajd.d2R = d2R;
trajd.Omega = Omega;
trajd.dOmega = dOmega;
trajd.pos = pos;
trajd.vel = vel;
trajd.acc = acc;
trajd.f = f;
trajd.M = M;
trajd.F = fb3;
end