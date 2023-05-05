function [H, k, d, V] = quadratic_dclf_const(dt, R, Om, Rc, Omc, gamma, c, epsilon,  inertia, inertia_inv, inertia_scaled)


%% computing error

Omhat = hat(Om);

% compute SO(3) error
eR = 0.5*vee(Rc'*R-R'*Rc);
eOmega = Om-R'*Rc*Omc;


%% computing lyapunov function at current time-step
V = eOmega'*inertia_scaled*eOmega/2 + epsilon*(eR'*eOmega)+c * (eR'*eR)/2;
    
%% integrated dynamics by 1 step

R2 = R*rodriguesRotation((Om*dt));
eR2 = 0.5*vee(Rc'*R2-R2'*Rc);

%% computing the quadratic constraint
b = 0.5*c*(eR2'*eR2);
rho = Om - R2'*Rc*Omc;
tmp = epsilon*eR2';

H_tilde = inertia_scaled*dt*dt;
k_tilde = (dt*rho'*inertia_scaled+ dt*tmp);
d_tilde = b+ 0.5*rho'*inertia_scaled*rho + tmp*rho + (gamma-1)*V;

H2 = inertia_inv'*H_tilde*inertia_inv;
OmJOm = Omhat*inertia*Om;
k2 = k_tilde*inertia_inv -OmJOm'*H2;
d = 0.5*OmJOm'*H2*OmJOm -k_tilde*inertia_inv*OmJOm +d_tilde;

H = blkdiag(H2,0);
k = [k2, -1]';

end


function [R] = rodriguesRotation(w)
    th = sqrt(w(1)*w(1)+w(2)*w(2)+w(3)*w(3));
    if th < 1e-6
        k = [0;0;1];
    else
        k = w/th;
    end
    K = hat(k);

    R = eye(3) + sin(th)*K + (1-cos(th))*K*K;
end


