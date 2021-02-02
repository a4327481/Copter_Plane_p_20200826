%fft tools    
function fft_function(x,y)

    Fs = floor(1/mean(diff(x)));                    % Sampling frequency
    L = length(y);                     % Length of signal
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
    Y = fft(y,NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);
    Spectrum = 100*2*abs(Y(1:NFFT/2+1));
    plot(f,Spectrum)