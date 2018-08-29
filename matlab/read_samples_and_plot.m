clear all;close all;clc

filename = "usrp_samples.dat";
fileID = fopen(filename);

FFT_LENGTH = 2048;

SUBFRAME_SIZE = 5760;

N = 4000*SUBFRAME_SIZE;

offset = 2*4*(2e7+0e6); % 2 samples to form I+j*Q times 4 bytes per component, i.e., I or Q.

fseek(fileID, offset, 'bof');

A = fread(fileID, 2*N, 'float');

fclose(fileID);

re = A(1:2:end);
im = A(2:2:end);

signal = complex(re,im);

h1 = figure(1);
title('IQ samples')
plot(real(signal))
saveas(h1,'iq_samples.png')

h2 = figure(2);
title('Spectrogram')
fs = 23.04e6;
overlap = 512;
spectrogram(signal,kaiser(FFT_LENGTH,5), overlap ,FFT_LENGTH , fs ,'centered');
saveas(h2,'spectrogram.png')
