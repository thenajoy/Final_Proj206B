clear;

%% symbols
syms dt c2 e2 real

J = sym('J', [3,3], 'real');
iJ = sym('iJ', [3,3], 'real');

Ri = sym('Ri', [3,3], 'real');
Rdi = sym('Rdi', [3,3], 'real');
Omi = sym('Omi', [3,1], 'real');
Omdi = sym('Omdi', [3,1], 'real');

Rj = sym('Rj', [3,3], 'real');
Rdj = sym('Rdj', [3,3], 'real');
Omdj = sym('Omdj', [3,1], 'real');

M = sym('M', [3,1], 'real');

eRi = 0.5*vee(Rdi'*Ri-Ri'*Rdi);
eOmi = Omi - Ri'*Rdi*Omdi;
eRj = 0.5*vee(Rdj'*Rj-Rj'*Rdj);

dOmi = iJ*(M)-iJ*(hat(Omi)*J*Omi);
Omj = Omi + dt*dOmi


%% math



