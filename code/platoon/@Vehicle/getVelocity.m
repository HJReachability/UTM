function [v, vhist] = getVelocity(obj)
% vel = getPosition(obj)
%     returns the velocity and optinally the velocity history of the vehicle

if ~isempty(obj.vdim)
  v = obj.x(obj.vdim);
  vhist = obj.xhist(obj.vdim, :);
end

% Plane is a special case
if isa(obj, 'Plane')
  if isscalar(obj.vrange)
    v = obj.vrange;
    vhist = v * ones(size(obj.x, 2));
  elseif isempty(obj.u)
    v = mean(obj.vrange);
    vhist = mean(obj.vrange);    
  else
    v = obj.u(1);
    vhist = obj.uhist(1,:);    
  end
end

% If the velocity is a scalar, and there's a heading dimension, then we need to
% compute the velocity from speed and heading
if isscalar(v) && ~isempty(obj.hdim)
  [h, hhist] = obj.getHeading();
  v = v * [cos(h); sin(h)];
  vhist = [vhist.*cos(hhist); vhist.*sin(hhist)];
end

end