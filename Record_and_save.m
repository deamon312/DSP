%Create an audiorecorder object for CD-quality audio in stereo, and view its properties:

recorder = audiorecorder(44100, 16, 1,1);


%Collect a sample of your speech with a microphone, and plot the signal data:

% Record your voice for 5 seconds.
recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, 5);
disp('End of Recording.');
% 
% % Play back the recording.
play(recObj);
% 
% % Store data in double-precision array.
myRecording_Signal = getaudiodata(recObj);
% 
%% Plot the waveform.
plot(myRecording_Signal);
audiowrite('Signal.wav',myRecording_Signal,44100)