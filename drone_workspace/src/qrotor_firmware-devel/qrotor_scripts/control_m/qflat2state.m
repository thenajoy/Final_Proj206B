function[state] = qflat2state(m, J, flats)
g = 9.80665;

state = struct('p', zeros(3,1), ...
                'v', zeros(3,1),...
                'a', zeros(3,1),...
                'da', zeros(3,1),...
                'd2a', zeros(3,1),...
                'R', eye(3),...
                'Omega', zeros(3,1),...
                'dOmega', zeros(3,1),...
                'F', zeros(3,1),...
                'f', 0,...
                'M', zeros(3,1));
%%
state.p = flats.p;
state.v = flats.v;
state.a = flats.a;
state.da = flats.da;
state.d2a = flats.d2a;

fb3 = m * (state.a + [0; 0; g]);
norm_fb3 = norm(fb3) ;
f = norm_fb3 ;
b3 = fb3 / norm_fb3 ;
b3_b1d = cross(b3, flats.b1) ;
norm_b3_b1d = norm(b3_b1d) ;
b1 = - cross(b3, b3_b1d) / norm_b3_b1d ;
b2 = cross(b3, b1) ;
R = [b1 b2 b3] ;

dfb3 = m * state.da;
dnorm_fb3 = dot(fb3, dfb3) / norm_fb3 ;
db3 = (dfb3*norm_fb3 - fb3*dnorm_fb3) / norm_fb3^2 ;
db3_b1d = cross(db3, flats.b1) + cross(b3, flats.db1) ;
dnorm_b3_b1d = dot(b3_b1d, db3_b1d) / norm_b3_b1d ;
db1 = (-cross(db3,b3_b1d)-cross(b3,db3_b1d) - b1*dnorm_b3_b1d) /norm_b3_b1d ;
db2 = cross(db3, b1) + cross(b3, db1) ;
dR = [db1 db2 db3] ;
Omega_hat = R' * dR;
Omega = vee(Omega_hat) ;

d2fb3 = m*state.d2a;
d2norm_fb3 = (dot(dfb3, dfb3)+dot(fb3, d2fb3) - dnorm_fb3*dnorm_fb3)/norm_fb3 ;
d2b3 = ( (d2fb3*norm_fb3+dfb3*dnorm_fb3 - dfb3*dnorm_fb3-fb3*d2norm_fb3)*norm_fb3^2- db3*norm_fb3^2*2*norm_fb3*dnorm_fb3) / norm_fb3^4 ;
d2b3_b1d = cross(d2b3, flats.b1)+cross(db3, flats.db1) + cross(db3, flats.db1)+cross(b3, flats.d2b1) ;
d2norm_b3_b1d = ( (dot(db3_b1d,db3_b1d)+dot(b3_b1d,d2b3_b1d))*norm_b3_b1d - dot(b3_b1d, db3_b1d)*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
d2b1 = ( (-cross(d2b3,b3_b1d)-cross(db3,db3_b1d) - cross(db3,db3_b1d)-cross(b3,d2b3_b1d) - db1*dnorm_b3_b1d-b1*d2norm_b3_b1d )*norm_b3_b1d - db1*norm_b3_b1d*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
d2b2 = cross(d2b3, b1)+cross(db3, db1) + cross(db3, db1)+cross(b3, d2b1) ;
d2R = [d2b1 d2b2 d2b3] ;
dOmega = vee( dR' * dR + R'*d2R ) ; %vee( dR'*dR + R'*d2R, true ) ;

M = J*dOmega + cross(Omega, J*Omega) ;

state.R = R;
state.Omega = Omega;
state.dOmega = dOmega;
state.F = fb3;
state.f =f;
state.M = M;




end