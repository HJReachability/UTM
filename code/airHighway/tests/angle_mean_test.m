function angle_mean_test()
% Create a bunch of angles near pi
N = 100;
noise = pi/4;

mean = pi;
[raw_thetas, thetas] = gen_angles(mean, noise, N);

figure;
plot(raw_thetas, 'k-'); hold on
plot(thetas, 'b.'); 
refline(0, angle_mean(thetas,noise))
title(['Horizontal line should be near ' num2str(mean)])

% Create a bunch of angles away from pi
mean = -pi+noise + 2*(pi-noise)*rand;
[raw_thetas, thetas] = gen_angles(mean, noise, N);

figure;
plot(raw_thetas, 'k-'); hold on
plot(thetas, 'b.'); 
refline(0, angle_mean(thetas))
title(['Horizontal line should be near ' num2str(mean)])
end

function [raw_thetas, thetas] = gen_angles(mean, noise, N)
% Generates N angles with some mean and noise around the mean
% Outputs both the raw angles and the wrapped angles around pi
raw_thetas = mean-noise + 2*noise*rand(N,1);
thetas = raw_thetas;
thetas(thetas>pi) = thetas(thetas>pi) - 2*pi;
end