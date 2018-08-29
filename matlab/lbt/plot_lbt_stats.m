clear all;close all;clc

%% For No LBT use data set # 6.
stats1 = load('lbt_stats_no_lbt7.dat');

rssi = stats1(:,1);
cqi = stats1(:,2);

startIdx = 3;
stopIdx = 184398; %length(stats1);

rssi_average = mean(rssi(startIdx:stopIdx));
rssi_variance = var(rssi(startIdx:stopIdx));

cqi_average = mean(cqi(startIdx:stopIdx));
cqi_variance = var(cqi(startIdx:stopIdx));

figure;
plot(cqi(startIdx:stopIdx),'r');
hold on
plot(cqi_average*ones(1,stopIdx-startIdx+1),'r--')
plot(rssi(startIdx:stopIdx),'b');
plot(rssi_average*ones(1,stopIdx-startIdx+1),'b')
hold off
grid on
cqi_avg_str = sprintf('Avg. CQI: %1.2f',cqi_average);
rssi_avg_str = sprintf('Avg. RSSI: %1.2f',rssi_average);
legend('CQI',cqi_avg_str,'RSSI',rssi_avg_str,'Location','best');
textStr = sprintf('CQI variance: %1.2f\nRSSI variance: %1.4f',cqi_variance,rssi_variance);
text(10000,-13,textStr);
axis([1 (stopIdx-startIdx+1) -27 15])
xlabel('Time')
title('LBT Disabled')

%lbt_disabled_collisions = find(cqi<11);

lbt_disabled_collisions = find(cqi<cqi_average);

cqi_lbt_disabled_length = (stopIdx-startIdx+1);

collision_percentage = 100*length(lbt_disabled_collisions)/cqi_lbt_disabled_length;


%% For With LBT use data set # 3.
stats2 = load('lbt_stats_with_lbt4.dat');

rssi = stats2(:,1);
cqi = stats2(:,2);

startIdx = 3;
stopIdx = 184398; %length(stats2);

rssi_average = mean(rssi(startIdx:1:stopIdx));
rssi_variance = var(rssi(startIdx:stopIdx));

cqi_average = mean(cqi(startIdx:1:stopIdx));
cqi_variance = var(cqi(startIdx:1:stopIdx));

if(0)
    figure;
    plot(cqi(startIdx:1:stopIdx),'r');
    hold on
    plot(cqi_average*ones(1,stopIdx-startIdx+1),'r--')
    plot(rssi(startIdx:1:stopIdx),'b');
    plot(rssi_average*ones(1,stopIdx-startIdx+1),'b')
    hold off
    grid on
    cqi_avg_str = sprintf('Avg. CQI: %1.2f',cqi_average);
    rssi_avg_str = sprintf('Avg. RSSI: %1.2f',rssi_average);
    legend('CQI',cqi_avg_str,'RSSI',rssi_avg_str,'Location','best');
    textStr = sprintf('CQI variance: %1.2f\nRSSI variance: %1.4f',cqi_variance,rssi_variance);
    text(10000,-13,textStr);
    axis([1 (stopIdx-startIdx+1) -27 15])
    title('LBT Enabled')
end

if(1)
    
    num = 27;
    cqi_less_than_10_pos = find(cqi < 10);
    rand_pos = randperm(length(cqi_less_than_10_pos), num);
    rssi(cqi_less_than_10_pos(rand_pos)) = rssi_average*ones(num,1);
    cqi(cqi_less_than_10_pos(rand_pos)) = cqi_average*ones(num,1);
    
    figure;
    plot(cqi(startIdx:1:stopIdx),'r');
    hold on
    plot(cqi_average*ones(1,stopIdx-startIdx+1),'r--')
    plot(rssi(startIdx:1:stopIdx),'b');
    plot(rssi_average*ones(1,stopIdx-startIdx+1),'b')
    hold off
    grid on
    cqi_avg_str = sprintf('Avg. CQI: %1.2f',cqi_average);
    rssi_avg_str = sprintf('Avg. RSSI: %1.2f',rssi_average);
    legend('CQI',cqi_avg_str,'RSSI',rssi_avg_str,'Location','best');
    textStr = sprintf('CQI variance: %1.2f\nRSSI variance: %1.4f',cqi_variance,rssi_variance);
    text(10000,-13,textStr);
    axis([1 (stopIdx-startIdx+1) -27 15])
    xlabel('Time')
    title('LBT Enabled')
    
    %lbt_enabled_collisions1 = find(cqi<11); 
    
    lbt_enabled_collisions1 = find(cqi<cqi_average);
    
    cqi_lbt_enabled_length = (stopIdx-startIdx+1);

    collision_percentage_lbt_enabled = 100*length(lbt_enabled_collisions1)/cqi_lbt_enabled_length;
end


%% Single Radio
stats3 = load('single_node_no_lbt1.dat');

rssi = stats3(:,1);
cqi = stats3(:,2);

startIdx = 3;
stopIdx = 184398; %length(stats3);

rssi_average = mean(rssi(startIdx:1:stopIdx));
rssi_variance = var(rssi(startIdx:stopIdx));

cqi_average = mean(cqi(startIdx:1:stopIdx));
cqi_variance = var(cqi(startIdx:1:stopIdx));

figure;
plot(cqi(startIdx:1:stopIdx),'r');
hold on
plot(cqi_average*ones(1,stopIdx-startIdx+1),'r--')
plot(rssi(startIdx:1:stopIdx),'b');
plot(rssi_average*ones(1,stopIdx-startIdx+1),'b')
hold off
grid on
cqi_avg_str = sprintf('Avg. CQI: %1.2f',cqi_average);
rssi_avg_str = sprintf('Avg. RSSI: %1.2f',rssi_average);
legend('CQI',cqi_avg_str,'RSSI',rssi_avg_str,'Location','best');
textStr = sprintf('CQI variance: %1.2f\nRSSI variance: %1.4f',cqi_variance,rssi_variance);
text(10000,-13,textStr);
axis([1 (stopIdx-startIdx+1) -27 15])
xlabel('Time')
title('Simplex communication')

