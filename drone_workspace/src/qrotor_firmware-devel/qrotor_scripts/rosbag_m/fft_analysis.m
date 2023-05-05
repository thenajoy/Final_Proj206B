function [f, power] = fft_analysis(x, fs)

%    y = fft(x);
%    n = length(x);          % number of samples
%     f = (0:n-1)*(fs/n);     % frequency range
%     power = abs(y).^2/n;    % power of the DFT

    q_bar = x;
    N = length(q_bar) ;
    
    Q_bar = fft(q_bar, N) ;
    Q_bar = fftshift(Q_bar) ;
    f_ind = [-N/2:N/2-1] ;
    freq_span = f_ind/N * fs ;
    stem(freq_span, abs(Q_bar), ':.') ; 

    power = Q_bar;
    f = freq_span;
    
%     figure;
%     plot(f,power)
%     xlabel('Frequency')
%     ylabel('Power')
    
end