clear all;close all;clc

%filename = '/home/zz4fap/work/timed_comands_for_discontinous_tx/scatter/build/tx_side_assessment_5MHz.dat';
filename = '/home/zz4fap/work/mcf_tdma/scatter/build/tx_side_assessment_20MHz_with_W_flag_subframe_0.dat';

fileID = fopen(filename);

phy_bw_to_be_used = 6;

SUBFRAME_LENGTH = [1920 3840 5760 11520 15360 23040];
numFFT = [128 256 384 768 1024 1536];       % Number of FFT points
deltaF = 15000;                             % Subcarrier spacing
numRBs = [6 15 25 50 75 100];               % Number of resource blocks
rbSize = 12;                                % Number of subcarriers per resource block
cpLen_1st_symb  = [10 20 30 60 80 120];     % Cyclic prefix length in samples

signal_buffer = complex(zeros(1,SUBFRAME_LENGTH(phy_bw_to_be_used)),zeros(1,SUBFRAME_LENGTH(phy_bw_to_be_used)));

value = fread(fileID, 2, 'float');
idx = 0;
while ~feof(fileID)
    
    idx = idx + 1;
    signal_buffer(idx) = complex(value(1),value(2));
    
    value = fread(fileID, 2, 'float');
end
fclose(fileID);


symbol0 = signal_buffer(121:120+1536);

ofdm_symbol0 = (1/sqrt(1536))*fftshift(fft(symbol0,1536));

%ofdm_symbol0 = (1/sqrt(1536))*(fft(symbol0,1536));


plot(10*log10(abs(ofdm_symbol0)),'b-')

if(0)
hFig = figure('Position', figposition([1.5*46 1.5*50 1.5*30 1.5*30]), 'MenuBar', 'none');
%axis([-0.5 0.5 -50 0]);
hold on; 
grid on
xlabel('Normalized frequency');
ylabel('PSD (dBW/Hz)')
[psd_ofdm,f_ofdm] = periodogram(signal_buffer, rectwin(length(signal_buffer)), numFFT(phy_bw_to_be_used)*2, 1, 'centered'); 
plot(f_ofdm,10*log10(psd_ofdm));
end

