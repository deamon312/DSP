%% Load the signal
[y,Fs] = audioread('Signal.wav');
[noise,Fs] = audioread('Noise.wav');

%% Add noise to the signal
signal = y;


%noise = randn(1000,1);
filt = dsp.FIRFilter;
filt.Numerator = fir1(11,0.3);
%fnoise = filt(noise);
d = signal + noise;

coeffs = (filt.Numerator).'-0.01; % Set the filter initial conditions.
mu = 0.01; % Set the step size for algorithm updating.
%coeffs = ones(1,21);
lms = dsp.LMSFilter(12,'StepSize',mu,'InitialConditions',coeffs);
[~,e,w] = lms(noise,d);
subplot(2,1,1)
plot(e);
subplot(2,1,2)
plot(signal);
title('Noise Cancellation by the Sign-Data Algorithm');
legend('Actual signal','Result of noise cancellation',...
       'Location','NorthEast');
xlabel('Time index')
ylabel('Signal values')

figure()
stem(coeffs)
hold on
stem(w)