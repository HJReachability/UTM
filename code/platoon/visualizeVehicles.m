function visualizeVehicles(vehicles)
% function visualizeVehicles(vehicles)
%
% Visualizes the properties of vehicles stored in a cell structure
%
% Input:  vehicles - cell structure containing vehicle objects
%
% Mo Chen, 2015-07-08

%% Constant Parameters
% y position of boxes
box_posy = 0; 

% Box size
boxx = 3;
boxy = 4;

% x Position of each box
node_start = 0; 
node_diff = 10;
box_posx = node_start:node_diff:(length(vehicles)-1)*node_diff;

% Size of arrows representing pointers
arrow_size = 0.5*(node_diff-boxx);

% Offset for text inside the boxes
text_osx = 0.1*boxx;
text_osy = 0.5*boxy; 

% FQ pointer text and arrow offsets (with respect to box_pos)
FQ_text_osx = -1 * boxx;
FQ_text_osy = 0.8 * boxy;
FQ_arrow_osy = 0.7 * boxy;

% BQ pointer text and arrow offsets (with respect to box_pos)
BQ_text_osx = 1.05 * boxx;
BQ_text_osy = 0.3 * boxy;
BQ_arrow_osx = 1 * boxx;
BQ_arrow_osy = 0.4 * boxy;

% Leader pointer text and arrow offsets (with respect to box_pos)
LQ_text_osx = -1 * boxx;
LQ_text_osy = 0.5 * boxy;
LQ_arrow_osy = 0.4 * boxy;

%% Draw Boxes
nl = char(10); % new line character

hs = [];
for i = 1:length(vehicles)
    h = rectangle('Position', [box_posx(i) box_posy boxx boxy]); hold on
    hs = [hs h];
    
    hs(i).LineWidth = 2;
    
    % Inside the box
    text_box = ['vehicle.ID=' num2str(vehicles{i}.ID) nl ...
        '  .q=''' vehicles{i}.q '''' nl ...
        '  .idx=' num2str(vehicles{i}.idx) nl ...
        '  .idxJoin=' num2str(vehicles{i}.idxJoin)];
    text(box_posx(i)+text_osx, box_posy+text_osy, text_box);
    
    % FQ pointer text and arrow
    if ~isempty(vehicles{i}.FQ)
        text_box_FQ = ['.FQ.ID=' num2str(vehicles{i}.FQ.ID)];
    else
        text_box_FQ = ['.FQ.ID='];
    end
    text(box_posx(i)+FQ_text_osx, box_posy+FQ_text_osy, text_box_FQ);
    
    quiver(box_posx(i), box_posy+FQ_arrow_osy, -arrow_size, 0, ...
        'color', 'k');
    
    % BQ pointer text and arrow
    if ~isempty(vehicles{i}.BQ)
        text_box_BQ = ['.BQ.ID=' num2str(vehicles{i}.BQ.ID)];
    else
        text_box_BQ = ['.BQ.ID='];
    end
    text(box_posx(i)+BQ_text_osx, box_posy+BQ_text_osy, text_box_BQ);
    
    quiver(box_posx(i)+BQ_arrow_osx, box_posy+BQ_arrow_osy, ...
        arrow_size, 0, 'color', 'k');
    
    % Leader pointer text and arrow
    if ~isempty(vehicles{i}.Leader)
        text_box_LQ = ['.Leader.ID=' num2str(vehicles{i}.Leader.ID)];
    else
        text_box_LQ = ['.Leader.ID='];
    end
    text(box_posx(i)+LQ_text_osx, box_posy+LQ_text_osy, text_box_LQ);
    
    quiver(box_posx(i), box_posy+LQ_arrow_osy, -arrow_size, 0, ...
        'color', 'k');
    
    % Plot limits and shape
    axis equal
    xlim([box_posx(1)-boxx box_posx(end)+2*boxx])
end
% 
% figure_pos = [gcf.Position(1:2) 360*length(vehicles) 800]; % window size
% gcf.Position = figure_pos;
end % end function