function u = RSControl(obj, RS)
% u = RSControl(obj, reach, g, tau)

u = [];
for i = 1:length(RS.tau)
  if eval_u(RS.g, RS.data(:,:,:,i), obj.x) <= 0
    P = extractCostates(RS.g, RS.data(:,:,:,i));
    p = calculateCostate(RS.g, P, obj.x);
    u = (p(3) >= 0) * obj.wMin + (p(3) < 0) * obj.wMax;
    return
  end
end

end