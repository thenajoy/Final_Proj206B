
syms nx ny nz real
xiv = sym('xiv', [3,1], 'real');
xix = sym('xix', [3,1], 'real');


eta = [nx; ny; nz];

A = [[hat(eta), xiv, xix];
        zeros(2,5)];
   
expA = eye(5);
for i = 1:2
    expA = expA + (1/factorial(i))*A^i;
end






