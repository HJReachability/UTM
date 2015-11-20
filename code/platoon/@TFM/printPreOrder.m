function printPreOrder(obj)
% printPreOrder(obj)
% Method of TFM class

obj.printInfo;
for i = 1:length(obj.hws)
  obj.hws{i}.printPreOrder;
end
end