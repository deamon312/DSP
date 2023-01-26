%Create an audiorecorder object for CD-quality audio in stereo, and view its properties:

recorder = audiorecorder(44100, 16, 1,1);


%Collect a sample of your speech with a microphone, and plot the signal data:

% Record your voice for 5 seconds.
disp('Start speaking.')
recordblocking(recorder, 5);
disp('End of Recording.');
%% Play back the recording.
play(recorder);
% 
% % Store data in double-precision array.
myRecording_Signal = getaudiodata(recorder);
% 
%% Plot the waveform.
plot(myRecording_Signal(80000:end));
audiowrite('Noise1.wav',myRecording_Signal(80000:end),44100)
%%
FFT(44100,myRecording_Signal(80000:end),1);
