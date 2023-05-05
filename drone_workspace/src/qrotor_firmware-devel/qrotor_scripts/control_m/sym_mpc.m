%%
clear;

%%
N = 10;

x0 = sym('x0', 'real', [6,1]);
x = sym('x', 'real', [6, N]);
u = sym('x', 'real', [3, N]);
xd = sym('xd', 'real', [6, N]);
ud = sym('xd', 'real', [3, N]);


Q = sym('Q', 'real', [6,6]);


