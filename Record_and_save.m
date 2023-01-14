%Create an audiorecorder object for CD-quality audio in stereo, and view its properties:

recObj = audiorecorder(44100, 16, 1,3);
get(recObj)

%Collect a sample of your speech with a microphone, and plot the signal data:

% Record your voice for 5 seconds.
%recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, 5);
disp('End of Recording.');

% Play back the recording.
play(recObj);

% Store data in double-precision array.
myRecording_Noise2 = getaudiodata(recObj);

% Plot the waveform.
plot(myRecording_Noise2);
audiowrite('Noise.wav',myRecording_Noise2,44100)