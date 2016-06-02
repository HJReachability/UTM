function [s, shist] = getSpeed(obj)
% function getSpeed(obj)
%     returns the speed and optionally speed history of the vehicle

[v, vhist] = obj.getVelocity;
s = norm(v);

shist = zeros(1, size(vhist, 2));
for i = 1:size(vhist, 2)
  shist(i) = norm(vhist(:,i));
end


end