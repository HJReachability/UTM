function plotPosVelx(obj, color)
% Plots position and velocity on the same plot, for the x direction

[p, phist] = obj.getPosition;
[v, vhist] = obj.getVelocity;

if isempty(obj.hpxvxhist) || ~isvalid(obj.hpxvxhist)
  % If no graphics handle has been created, create it. Use custom color
  % if provided.  
  if nargin<2
    obj.hpxvxhist = plot(phist(1,:), vhist(1,:), ':');
  else
    obj.hpxvxhist = plot(phist(1,:), vhist(1,:), ':', 'color', color);
  end
  hold on
else
  % Otherwise, simply update the graphics handles
  obj.hpxvxhist.XData = phist(1,:);
  obj.hpxvxhist.YData = vhist(1,:);
end

%% Plot current position and velocity using an arrow
if isempty(obj.hpxvx) || ~isvalid(obj.hpxvx)
  % If no graphics handle has been created, create it with the specified
  % color. Use default color if no color is provided.
  if nargin<2
    obj.hpxvx = plot(p(1), v(1), 'o');
  else
    obj.hpxvx = plot(p(1), v(1), 'o', 'color', color);
  end
  hold on
  
  obj.hpxvx.Color = obj.hpxvxhist.Color;
  obj.hpxvx.MarkerFaceColor = obj.hpxvxhist.Color;
  obj.hpxvx.MarkerSize = 6;
else
  % Otherwise, simply update graphics handles
  obj.hpxvx.XData = p(1);
  obj.hpxvx.YData = v(1);
end

end