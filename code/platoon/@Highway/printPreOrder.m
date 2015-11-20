function printPreOrder(obj)
% printPreOrder(obj)
% method of Highway class

obj.printInfo;
for i = 1:length(obj.ps)
  obj.ps{i}.printPreOrder;
end
end