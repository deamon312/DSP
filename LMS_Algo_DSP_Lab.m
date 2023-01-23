%% In this model we will show you the process of canceling noise on a noisy 
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
% Load the signal and the corrupting factor of the signal
[y,~] = audioread('Signal.wav');
[noise,Fs] = audioread('Noise.wav');
signal =y;
% Set the noise as a random configuration
index = randi(numel(noise) - numel(y) + 1,1,1);
noiseSegment = noise(index:index + numel(y) - 1);
% Calculate the power components of the siganls
speechPower = sum(signal.^2);
noisePower = sum(noiseSegment.^2);
% Set the factor 'y1' of the system - signal + random noise
d = y + sqrt(speechPower/noisePower)*noiseSegment;

%% Plot corrupted signal ,Noise free signal and Noise signal
figure(1)
subplot(3,1,1)
plot(signal);
title('Noise free signal');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,2)
plot(noise);
title('Noise signal');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,3)
plot(d);
title('Corrupted signal');
xlabel('Time[s]');
ylabel('Amplitude[V]');

%% LMS Adapt Filter
% Set the step size for algorithm updating.
mu = 0.01;
% Filter length (num of taps)
M = 16; 
% Set the start point of the weights of the adaptiv filter
coeffs = zeros(1,M);
% Create struct in a new variable (external funciton)
S = LMSinit(zeros(M,1),mu);
% Perform LMS-algo. + export set of weights and coeffs(external funciton)
[~,e,S] = LMSadapt(noise,d,S);
%lms_nonnormalized = dsp.LMSFilter(M,'StepSize',mu,...
%    'Method','LMS','InitialConditions',coeffs);
%[~,e,w] = lms_nonnormalized(noise,d);
e = e';
w = S.coeffs;

%% Figure of three 'time X amp' plots
figure(2)
subplot(3,1,1)
plot(abs(e-signal));% Filt.effectiveness (wieghted signal-origin signal)
title('Indication of the effectiveness of the LMS');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,2)
plot(signal);
title('Original signal in time domain');
xlabel('Time[s]');
ylabel('Amplitude[V]');

subplot(3,1,3)
plot(e);
title('Filtered signal');
xlabel('Time[s]');
ylabel('Amplitude[V]');

% Figure of signal and filter result combined
figure(3)
plot([e,signal]);% Filt.result and original siganl for comparison
legend('Result of noise cancellation','Actual signal');
title('Indication of the effectiveness of the LMS');
xlabel('Time[s]');
ylabel('Amplitude[V]');

% Figure of the coeffs and weights in one stem
figure(4)
stem(coeffs)
hold on
stem(w)
legend('Initial Weights','Adapted Weights');
title('Stem figure of the weights of the FIR sequence');
xlabel('Taps');
ylabel('Coeff');

% Figure of the FFT of the signals 'amp X freq'
figure(5)
limit1 = [-4e3,4e3];% Relevant spectrum of regular speech frequency
subplot(4,1,1)
[FFT_signal_amp,FFT_freq] = FFT(Fs,signal,0);
plot(FFT_freq,FFT_signal_amp)
xlim(limit1)
title('origin signal in frequency domain');
xlabel('Frequency[f]');
ylabel('Amplitude[V]');

subplot(4,1,2)
[FFT_amp,FFT_freq] = FFT(Fs,noise,0);
plot(FFT_freq,FFT_amp)
xlim(limit1)
title('original noise in freq domain');
xlabel('Frequency[f]');
ylabel('Amplitude[V]');

subplot(4,1,3)
[FFT_amp,FFT_freq] = FFT(Fs,d,0);
plot(FFT_freq,FFT_amp)
xlim(limit1)
title('noisy signal in frequency domain');
xlabel('Frequency[f]');
ylabel('Amplitude[V]');

subplot(4,1,4)
[FFT_filt_amp,FFT_freq] = FFT(Fs,e,0);
plot(FFT_freq,FFT_filt_amp)
xlim(limit1)
title('filter results in frequency domain');
xlabel('Frequency[f]');
ylabel('Amplitude[V]');

std(FFT_signal_amp-FFT_filt_amp)/mean(FFT_signal_amp)
%%
sound(e,44100)
%%
sound(d,44100)
%%
sound(signal,44100)