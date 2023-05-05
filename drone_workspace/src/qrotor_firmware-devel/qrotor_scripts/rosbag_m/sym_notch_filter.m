clear;
%%
% th = pi *freq_ratio;
% nwc = 1-nw
syms th nw nwc s real;
notch_zeros = [cos(th)+1i*sin(th), cos(th)-1i*sin(th)];
notch_poles = simplify(nwc*notch_zeros);

% moving average filter coefficients
b_ = simplify((s-notch_zeros(1))*(s-notch_zeros(2)))

% autoregressive filter coefficients
a_ = simplify((s-notch_poles(1))*(s-notch_poles(2)))



