function u = RSControl(obj, BRS)
% u = RSControl(obj, BRS)

u = [];
for i = 1:length(BRS.tau)
  if eval_u(BRS.g, BRS.data(:,:,:,i), obj.x) <= 0
    P = extractCostates(BRS.g, BRS.data(:,:,:,i));
    p = calculateCostate(BRS.g, P, obj.x);
    u = (p(3) >= 0) * (-BRS.uMax) + (p(3) < 0) * BRS.uMax;
    return
  end
end

% Change this!

end