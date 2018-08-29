clear all;close all;clc

competition_bw = 20e6;

competition_center_freq = 1e9; %5e9;

phy_bandwidth = 4500000;

for channel=0:1:5
   
    channel_center_freq = (competition_center_freq - (competition_bw/2.0) + (phy_bandwidth/2.0) + channel*phy_bandwidth);
    fprintf(1,'Channel: %d - Frequency: %f\n',channel,channel_center_freq/1000000);
end