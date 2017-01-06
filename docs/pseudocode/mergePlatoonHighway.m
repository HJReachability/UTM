% Under platoon class
% Need to add highway class
% Need to add highway as a property of quadrotors and platoons

function u = mergeWithPlatoon(obj, other)

idx = mergeIdx(obj, other);

u = mergeControl(obj, other, idx);

end

function idx = mergeIdx(obj, other)

if obj.highway == other.highway
	idx = other.n+1;
end

end

function u = mergeControl(obj, other, idx)

if idx > other.n
	u = mergeWithQuadrotor( ... ); % use algorithm in there, but maybe indices may not be correct; need to make sure indices inside that function are treated properly... actually, that function is designed for only one quadrotor, so either make a new function for an entire platoon joining, or modify that function so that it's general (the latter shouldn't be too hard)
else
	% First update indices, then use controller
	for i = idx:other.n
		other.vehicle{i}.idx = other.vehicle{i}.idx + obj.n
		
		if error in new phantom position is small
		    u = mergeWithQuadrotor( ... );
		else
			perform holding pattern
		end
	end
end
	

end