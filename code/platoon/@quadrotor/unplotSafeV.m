function unplotSafeV(obj, other)
% function unplotSafeV(obj, other)
%
% Unplot safe region around other that obj must stay out of in order to
% be safe.
%
%
% Qie Hu, 2015-07-25

if ~isempty(obj.hsafeV{other.ID})
    set(obj.hsafeV{other.ID}, 'Visible','off');
end

end
