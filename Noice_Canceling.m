%% Load the signal
[y,Fs] = audioread('Signal.wav');
[noise,Fs] = audioread('Noise.wav');
%[y,Fs] = audioread('Mic2.wav');
%[noise,Fs] = audioread('Mic1.wav');
%% Add noise to the signal
%signal = y(1:150000);
%noise = noise(1:150000);
signal =y;
%noise = randn(1000,1);
filt = dsp.FIRFilter;
filt.Numerator = fir1(31,0.5);
fnoise = filt(noise);
d = signal+fnoise;

coeffs = (filt.Numerator).'-0.01; % Set the filter initial conditions.
mu = 0.0001; % Set the step size for algorithm updating.
%coeffs = zeros(1,32);
lms = dsp.LMSFilter(32,'StepSize',mu,'InitialConditions',coeffs);
[~,e,w] = lms(noise,d);
subplot(4,1,1)
plot(e);
subplot(4,1,2)
plot(signal);
subplot(4,1,3)
plot([e,signal]);
subplot(4,1,4)
plot(noise);
title('Noise Cancellation by the Sign-Data Algorithm');
legend('Actual signal','Result of noise cancellation',...
       'Location','NorthEast');
xlabel('Time index')
ylabel('Signal values')

figure()
stem(coeffs)
hold on
stem(w)
%%
sound(e,44100)

