function [FFT_amp,FFT_freq] = FFT(Fs,signal,display_plot)
    N = 2^nextpow2(10*length(signal));
    yf_singal = abs(fftshift(fft(signal,N)));
    FFT_freq = linspace(-Fs/2,Fs/2,N);
    
    Norm_factor =1/length(signal);
    FFT_amp =(Norm_factor*yf_singal);
%   FFT_amp =20*log10((Norm_factor*yf_singal)); 
    if display_plot==1
        figure('Name','Fast Fourier Transform')
        plot(FFT_freq,FFT_amp)
        xlim([-8e3,8e3])
    end
end
    
    