function unplotSafeV(obj)
% Unplot safeV of quadrotor

if ~isempty(obj.hsafeV)
%     obj.hsafeV.ZData = [];
    set(obj.hsafeV, 'Visible','off');
end

end
