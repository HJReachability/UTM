function printInfo(obj)
% printInfo(obj)
% method of Highway class

disp('Highway Info:')
disp(['  width = ' num2str(obj.width)])

pla_str = '  Platoons:';
for i = 1:length(obj.ps)
  pla_str = [pla_str ' ' num2str(obj.ps{i}.ID)];
end
disp(pla_str)
end