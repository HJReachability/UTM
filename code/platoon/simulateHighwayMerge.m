clear all
close all


% Create the highways 
hw1 = highway([-50 0], [50 0]);
hw2 = highway([-50 -50], [50 50]);

hws = [hw1 hw2];

figure
hwColors = lines(2);
for i = 1:length(hws)
    hws(i).hwPlot(hwColors(i,:,:)); hold on
end 

%
