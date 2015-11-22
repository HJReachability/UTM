function printPreOrder(obj)
% printPreOrder(obj)
% Method of Node class

obj.printInfo;
children = obj.getChildren;
for i = 1:length(children)
  children{i}.printPreOrder;
end
end