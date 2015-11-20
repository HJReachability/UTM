function printPreOrder(obj)
% printPreOrder(obj)
% Method of Platoon class

obj.printInfo;
for i = 1:length(obj.vehicles)
  if ~isempty(obj.vehicles{i})
    obj.vehicles{i}.printPreOrder;
  end
end
end