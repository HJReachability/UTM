function highwayPosTest(obj, z)
% function highwayPosTest(obj, z)
% Test routine for highwayPos; plots the results
%
% Mo Chen, 2015-05-25

disp('Testing highwayPos()')

% Call the function to be tested
[s, dist] = obj.highwayPos(z);

figure;
% Plot highway and closest point
plot([obj.z0(1) obj.z1(1)], [obj.z0(2) obj.z1(2)], 'k--'); hold on
fs = obj.fn(s);
plot(fs(1), fs(2), 'k*')

% Plot line segment between z and closest point
plot(z(1), z(2), 'r.')
plot([z(1) fs(1)], [z(2) fs(2)], 'r:')
axis equal
title(['distance = ' num2str(dist)])

% Check dot product
u = [z(1) - fs(1); z(2) - fs(2)];
v = [obj.z1(1) - fs(1); obj.z1(2) - fs(2)];
disp(['u dot v =' num2str(u' * v)])
end