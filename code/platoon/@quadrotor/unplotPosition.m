function unplotPosition(obj)
% Unplot position of quadrotor 

if ~isempty(obj.hpxpyhist)
    set(obj.hpxpyhist, 'Visible','off');
end
if ~isempty(obj.hpxpy)
    set(obj.hpxpy, 'Visible','off');
end

end