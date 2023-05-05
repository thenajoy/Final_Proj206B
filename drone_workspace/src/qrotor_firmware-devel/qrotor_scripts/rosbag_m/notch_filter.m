function [y] = notch_filter(x, fs, notch_freq, notch_width)
% source:https://dsp.stackexchange.com/a/1090
%
f0 = notch_freq;                % notch frequency
fn = fs/2;              % Nyquist frequency
freqRatio = f0/fn;      % ratio of notch freq. to Nyquist freq.

notchWidth = notch_width;       % width of the notch

% Compute zeros
notchZeros = [exp( sqrt(-1)*pi*freqRatio ), exp( -sqrt(-1)*pi*freqRatio )];

% Compute poles
notchPoles = (1-notchWidth) * notchZeros;
% 
% figure;
% zplane(notchZeros.', notchPoles.');

b = poly( notchZeros ); %  Get moving average filter coefficients
a = poly( notchPoles ); %  Get autoregressive filter coefficients

% figure;
% freqz(b,a,32000,fs)

% filter signal x
y = filter(b,a,x);


end