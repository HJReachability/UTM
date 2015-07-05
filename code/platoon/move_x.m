% Program: User Control Simulation Interface
% Author:  Jaime F. Fisac
% Version: 1.0
% Date:    2015/07/04
%
% This code generates a plot that gets updated according to some system
% dynamics and control input entered by the user through the keyboard.
%
% The functions below are modularly designed so as to be adapted to various
% simulations and demonstrations incorporating real-time human control.
%
% User input is processed through the figure object and therefore the
% corresponding information is stored and retreived using the figure
% itself, through guidata(). Any implemented code using this interface
% will need to keep this structure.
%
% To try this code, run and use either 'w','a','s','d' or the arrow keys
% while the figure is in focus to control the motion of the 'X' on the
% screen. Use keys 'x' or 'esc' to close the window and end the simulation.
% The dynamics can be modified by uncommenting some of the code in the
% "Main simulation loop", where indicated, or editing as desired.



function move_x()
% This function is a placeholder for more complex code comprising the main
% body of the desired simulation. The "Graphical setup" and "Define command
% keys" sections can be copied and modified for each particular
% application. The "Main simulation loop" section should be largely
% replaced by the appropriate code.

%% Graphical setup
% This section is an example and can be replaced by the actual simulation.

% Create a figure with the appropriate callbacks defined for key actions
S.fh = figure('units','pixels',...
              ...'position',[500 500 300 300],...
              'menubar','none',...
              'name','move_fig',...
              'numbertitle','off',...
              'keypressfcn',@cmd_press,...   % callback for key press
              'keyreleasefcn',@cmd_release); % callback for key release
          
plot(0,0,'bx','MarkerSize',16);
axis equal
axis([-10 10 -10 10]);
guidata(S.fh,S)

%% Define command keys
N_cmds = 5; % Here we are using 5 different controls (including 'quit')
S.cmdStatus = false(N_cmds,1);

% Allow alternative sets of control keys (for convenience)
S.cmdNames = {  'w', 'uparrow';
                's', 'downarrow';
                'a', 'leftarrow';
                'd', 'rightarrow';
                'x', 'escape'};
S.cmd.up    = 1;
S.cmd.down  = 2;
S.cmd.left  = 3;
S.cmd.right = 4;
S.cmd.quit  = 5;

guidata(S.fh,S) % store GUI information in figure object

%% Main simulation loop
% This section is an example and can be replaced by the actual simulation.

NotOver = true;
vehicle.x = 0;
vehicle.y = 0;

% This while loop should normally be replaced by a timer-dependent thread
while NotOver
    S = guidata(S.fh); % retrieve updated info from figure object
    % Control (Set Input): u_k = g_k(ctrl_k)
    if S.cmdStatus(S.cmd.up) && ~S.cmdStatus(S.cmd.down)
        ystep = 0.1;
    elseif S.cmdStatus(S.cmd.down) && ~S.cmdStatus(S.cmd.up)
        ystep = -0.1;
    else
        ystep = 0;
    end
    if S.cmdStatus(S.cmd.left) && ~S.cmdStatus(S.cmd.right)
        xstep = -0.1;
    elseif S.cmdStatus(S.cmd.right) && ~S.cmdStatus(S.cmd.left)
        xstep = 0.1;
    else
        xstep = 0;
    end
    if S.cmdStatus(S.cmd.quit)
        NotOver = false;
    end
    % Discrete-time Dynamics (Update State): x_{k+1} = f_k(x_k,u_k)
    % --------------------------------
    vehicle.x = ...
                ...0.99*...            % uncomment for stable attractor
                vehicle.x + ...
                ...0.1*vehicle.y + ... % uncomment for velocity drift
                xstep;
    vehicle.y = ...
                ...0.98*...            % uncomment for stable attractor
                vehicle.y + ...
                ystep;
    % --------------------------------
    % Plot
    X = S.fh.Children(1).Children(1);  % get handle to object (X)
    X.XData = vehicle.x;
    X.YData = vehicle.y;
    drawnow
end
fprintf('\nSimulation terminated by user.\n')
close(S.fh)

end



%% Key press and release callbacks
% These functions update the control input whenever keyboard status changes


function cmd_press(hObject,event)   
% This function updates the cmdStatus vector indicating what controls are
% being commanded by the user. It is automatically called when a key is
% pressed down on the keyboard.

S = guidata(hObject); % retrieve updated info from figure object
E = event;

S.cmdStatus = (any( strcmp(E.Key, S.cmdNames),2 ) | S.cmdStatus);
guidata(S.fh,S); % store update info in figure object

end


function cmd_release(hObject,event)
% This function updates the cmdStatus vector indicating what controls are
% being commanded by the user. It is automatically called when a key is
% released on the keyboard.

S = guidata(hObject); % retrieve updated info from figure object
E = event;

S.cmdStatus = (~any( strcmp(E.Key, S.cmdNames),2 ) & S.cmdStatus);
guidata(S.fh,S); % store updated info in figure object

end
