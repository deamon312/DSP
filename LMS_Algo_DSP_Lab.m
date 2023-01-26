%% In this model we will show the process of canceling noise from corrupted 
% signal using an adaptive filter applied by LMS algorithem as an 
% implementation to an article called:
% 'Simulation for noise cancellation using LMS adaptive filter.'
%% The process included:
% 1. Import and export original unnoised and noise signal as a wav file.
% 2. Implement noise components on the signal file randomly.
% 3. Define original functions. functions of:
%       Creating a struct for a variable
%       LMS algorithm execution
%       FFT algorithm execution
% 4. Create an adaptive filter model and applying it on the signals
% 5. Graphs figures of all relevant signals including:
%       Algorithm efficiency feedback
%       All relevant signals
%       All the graphs showed in time domain and frequency domain
%       Stem figure of the coeffs and weights of the model


clc
clear all 
close all
% Load the free noise signal and noise
[y,Fs] = audioread('Signal.wav');
[noise,~] = audioread('Noise1.wav');
signal =y;
% Set the noise as a random configuration
index = randi(numel(noise) - numel(y) + 1,1,1);
noiseSegment = noise(index:index + numel(y) - 1);
% Calculate the power components of the siganls
% Set the noise power such that the signal-to-noise ratio (SNR)
speechPower = sum(signal.^2);
noisePower = sum(noiseSegment.^2);
% Define corrupted signal  with noise factor
d = y + sqrt(speechPower/noisePower)*noiseSegment;
%% Plot corrupted signal ,noise free signal and noise 
figure(1)
dt = 1/Fs; 
t = 0:dt:(length(y)*dt)-dt; % create time vector
subplot(3,1,1)
plot(t,signal);
title('Noise free signal');
xlabel('Time[s]');
ylabel('Amplitude');

subplot(3,1,2)
plot(t,noiseSegment);
title('Noise');
xlabel('Time[s]');
ylabel('Amplitude');

subplot(3,1,3)
plot(t,d);
title('Corrupted signal');
xlabel('Time[s]');
ylabel('Amplitude');

%% LMS Adapt Filter
% Set the step size for algorithm updating.
mu = 0.3;
% Filter length (num of taps)
M = 11; 
% Set the start point of the weights of the adaptiv filter
coeffs = zeros(1,M);
% Create struct in a new variable (external funciton)
S = LMSinit(zeros(M,1),mu);
% Perform LMS-algo. + export set of weights and coeffs(external funciton)
[~,e,S] = LMSadapt(noiseSegment,d,S);
% lms_nonnormalized = dsp.LMSFilter(M,'StepSize',mu,...
%      'Method','LMS','InitialConditions',coeffs);
% [~,e,w] = lms_nonnormalized(noiseSegment,d);
e = e';
w = S.coeffs;
%% Frequency Response of Adaptive filter
figure(2)
[h,f] = freqz(w,1,[],Fs);
subplot(2,1,1);
plot(f,20*log10(abs(h))); % we will use 20log10() for ploting the mag. response in dB 
title('Magnitude response')
grid on % turning the grid on
xlabel('Frequency(Hz)') 
ylabel('Magnitude(dB)')
subplot(2,1,2);
plot(f,rad2deg(angle(h))); 
title('Phase response')
grid on
xlabel('Frequency(Hz)')
ylabel('phase(degree)')
%% Spectogram
figure(7)
subplot(3,1,1)
spectrogram(y,128,120,[],Fs,'yaxis' );
title('Original signal');
subplot(3,1,2)
spectrogram(e,128,120,[],Fs,'yaxis' );
title('Filtered signal');
subplot(3,1,3)
spectrogram(noiseSegment,128,120,[],Fs,'yaxis' );
title('Noise signal');
%view(0,0)
%% Figure of three 'time X amp' plots
figure(3)
subplot(3,1,1)
plot(t,abs(e-signal));% Filt.effectiveness (wieghted signal-origin signal)
title('Indication of the effectiveness of the LMS');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,2)
plot(t,signal);
title('Original signal in time domain');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,3)
plot(t,e);
title('Filtered signal');
xlabel('Time[s]');
ylabel('Amplitude[V]');

% Figure of signal and filter result combined
figure(4)
plot(t,e,t,signal);% Filt.result and original siganl for comparison
legend('Result of noise cancellation','Actual signal');
title('Indication of the effectiveness of the LMS');
xlabel('Time[s]');
ylabel('Amplitude[V]');

% Figure of the coeffs and weights in one stem
figure(5)
stem(coeffs)
hold on
stem(w)
legend('Initial Weights','Adapted Weights');
title('Stem figure of the weights of the FIR sequence');
xlabel('Taps');
ylabel('Coeff');

%% Figure of the FFT of the signals 'amp X freq'
figure(6)
limit1 = [-4e3,4e3];% Relevant spectrum of regular speech frequency
subplot(4,1,1)
[FFT_signal_amp,FFT_freq] = FFT(Fs,signal,0);
plot(FFT_freq,FFT_signal_amp)
xlim(limit1)
title('origin signal in frequency domain');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,2)
[FFT_amp,FFT_freq] = FFT(Fs,noise,0);
plot(FFT_freq,FFT_amp)
xlim(limit1)
title('original noise in freq domain');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,3)
[FFT_amp,FFT_freq] = FFT(Fs,d,0);
plot(FFT_freq,FFT_amp)
xlim(limit1)
title('noisy signal in frequency domain');
xlabel('Frequency[Hz]');
ylabel('Amplitude');

subplot(4,1,4)
[FFT_filt_amp,FFT_freq] = FFT(Fs,e,0);
plot(FFT_freq,FFT_filt_amp)
xlim(limit1)
title('filter results in frequency domain');
xlabel('Frequency[Hz]');
ylabel('Amplitude');
%%
sound(e,44100)
%%
sound(d,44100)
%%
sound(signal,44100)