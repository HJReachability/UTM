function printBreadthFirst(obj)
queue = {obj};

while ~isempty(queue)
  % Print current element of the queue
  queue{1}.printInfo;
  
  % Add children to queue
  children = queue{1}.getChildren;
  for j = 1:length(children)
    if ~isempty(children{j})
      queue{length(queue)+1, 1} = children{j};
    end
  end
  
  % Pop the queue
  queue(1) = [];
end
end