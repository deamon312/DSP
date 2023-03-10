%% Ruslan Osmanov 327480026
%% Tal Haim 312464639
%In this model we will show the process of canceling noise from corrupted 
%signal using an adaptive filter applied by LMS algorithem as an 
%implementation to an article called:
%'Simulation for noise cancellation using LMS adaptive filter.'
%% The process included:
%1. Import and export noise free and noise signal as a wav file.
%2. Implement noise components on the signal file randomly.
%3. Plot corrupted ,noise free and noise signal
%4. Run LMS algorithm according to:
%Diffirent Length of FIR filter (M)
%Diffirent Step Size (mu)
%5. Plot frequency responce of the last itteration coefficents 
%6. Plot Spectogram of filtered ,noise free and corrupted signals
%7. Plots:
%Absolute error beetwen noise free and filtered signal
%Noise free signal
%Filtered signal
%8. Relative error beetwen original and filtered signal
%9. Plots of signal and filter result combined
%10. Plots of the coeffs and weights:
%Initial Weights,Adapted Final Weights
%Coefficient Trajectories       
%11. FFT:
%Noise free signal
%Filtered signal
%Corrupted signal
%Noise signal

close all
clear all 

%% Load noise free and noise signals
[signal,Fs] = audioread('Signal.wav');
[noise,~] = audioread('Noise1.wav');

% Set the noise as a random configuration
index = randi(numel(noise) - numel(signal) + 1,1,1);
noiseSegment = noise(index:index + numel(signal) - 1);
% To ensure that the noise is correlated, pass the noise through 
% a lowpass FIR filter and then add the filtered noise to the signal.
filt = dsp.FIRFilter;
filt.Numerator = fir1(11,0.8);
fnoise = filt(noiseSegment);
freqz(filt)
%fnoise = circshift(noiseSegment,11); % shift noise for correlated noise
% Calculate the power components of the siganls
speechPower = sum(signal.^2);
noisePower = sum(fnoise.^2);
noise_factor =sqrt(speechPower/noisePower); % snr

% Define corrupted signal with noise factor
d = signal + noise_factor*fnoise;
corrcoef(fnoise,noiseSegment)
corrcoef(signal,fnoise)
%% Plot corrupted ,noise free and noise signal
figure(1)
dt = 1/Fs; 
t = 0:dt:(length(signal)-1)*dt; % create time vector

subplot(3,1,1)
plot(t,signal);
title('Noise free signal');
xlabel('Time[s]');
ylabel('Amplitude');

subplot(3,1,2)
plot(t,noiseSegment);
title('Noise signal');
xlabel('Time[s]');
ylabel('Amplitude');

subplot(3,1,3)
plot(t,d);
title('Corrupted signal');
xlabel('Time[s]');
ylabel('Amplitude');
linkaxes([subplot(3,1,1) subplot(3,1,2) subplot(3,1,3)], 'xy');
%% LMS Adapt Filter
mu = input('Step size mu = '); % Set the step size
M = input('Length of sequence M = ');  % Filter length (num of taps)
model_info = strcat('\mu : ',string(mu) ,'  M : ',string(M));

% Initialization of weights 
coeffs = zeros(M,1); % column vector of init weights
S.coeffs = coeffs; % insert weights to struct
S.step = mu; % insert step size to the struct

% Perform LMS-algo 
[~,e,S] = LMSadapt(noiseSegment,d,S);
w = S.coeffs;
% lms_nonnormalized = dsp.LMSFilter(M,'StepSize',mu,...
%        'Method','LMS','InitialConditions',coeffs);
% [~,e,w] = lms_nonnormalized(noiseSegment,d);
%% Frequency Response of Adaptive filter
figure(2)
[h,f] = freqz(w,1,[],Fs);
subplot(2,1,1);
hold on
plot(f,20*log10(abs(h)),'DisplayName',model_info); % we will use 20log10() for ploting the mag. response in dB 
title('Magnitude response')
grid on % turning the grid on
xlabel('Frequency(Hz)') 
ylabel('Magnitude(dB)')
legend
subplot(2,1,2);
hold on
legend
plot(f,rad2deg(angle(h)),'DisplayName',model_info); 
title('Phase response')
grid on
xlabel('Frequency(Hz)')
ylabel('Phase(degree)')
hold off
%% Spectogram
figure(3)
subplot(3,1,1)
spectrogram(signal,128,120,[],Fs,'yaxis' );
title('Noise free signal');
subplot(3,1,2)
spectrogram(d,128,120,[],Fs,'yaxis' );
title('Corrupted signal');
subplot(3,1,3)
spectrogram(e,128,120,[],Fs,'yaxis' );
title(strcat('Filtered signal ',model_info));
linkaxes([subplot(3,1,1) subplot(3,1,2) subplot(3,1,3)], 'xy');
%view(0,0)

%% Time Domain plots
figure(4)
subplot(3,1,1)
hold on
err = num2str(norm(e-signal)/norm(signal)*100);
plot(t,e-signal,'DisplayName',strcat(model_info,'  err:',err,'%'));% Filt.effectiveness 
title('Error beetwen noise free and filtered signals');
xlabel('Time[s]');
ylabel('Amplitude');
legend
disp(['Relative error beetwen noise free and filtered signal :',num2str(norm(e-signal)/norm(signal)*100) ,' %'])
hold off

subplot(3,1,2)
plot(t,signal);
title('Noise free signal');
xlabel('Time[s]');
ylabel('Amplitude');

subplot(3,1,3)
plot(t,e);
title(strcat('Filtered signal  ',model_info));
xlabel('Time[s]');
ylabel('Amplitude');
linkaxes([subplot(3,1,1) subplot(3,1,2) subplot(3,1,3)], 'xy');
% Combined plot of noise free and filter signal
figure(5)
plot(t,e,t,signal);
legend('Filtered signal','Noise free signal');
title('Result of noise cancellation');
xlabel('Time[s]');
ylabel('Amplitude');

%% Plot of the coeffs and weights
figure(6)
subplot(2,1,1)
stem(coeffs)
hold on
stem(w)
legend('Initial Weights','Adapted Final Weights');
title(strcat('Coefficents of the FIR filter ',model_info));
xlabel('Taps');
ylabel('Coeff');
hold off

subplot(2,1,2)
nn = length(e);
plot(1:nn,S.W(:,1:nn))
title('Coefficient Trajectories');
xlabel('Iteration');
ylabel('Coeff');
legend(string(1:M),'Location','best')
legend('boxoff')
%% Frequency domain plots
figure(7)
limit = [-4e3,4e3];% Relevant spectrum of regular speech frequency
subplot(4,1,1)
[FFT_amp,FFT_freq] = FFT(Fs,signal,0);
plot(FFT_freq,FFT_amp)
xlim(limit)
title('Noise free signal');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,2)
[FFT_amp_n,FFT_freq] = FFT(Fs,noise,0);
plot(FFT_freq,FFT_amp_n)
xlim(limit)
title('Noise Signal');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,3)
[FFT_amp_d,FFT_freq] = FFT(Fs,d,0);
plot(FFT_freq,FFT_amp_d)
xlim(limit)
title('Corrupted signal');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,4)
[FFT_amp_filt,FFT_freq] = FFT(Fs,e,0);
plot(FFT_freq,FFT_amp_filt)
xlim(limit)
title(strcat('Filter signal  ',model_info));
xlabel('Frequency[Hz]');
ylabel('Amplitude');

linkaxes([subplot(4,1,1) subplot(4,1,2) subplot(4,1,3) subplot(4,1,4)], 'x');
%%
sound(e,44100)
%%
sound(d,44100)
%%
sound(signal,44100)
%%
sound(noise,44100)