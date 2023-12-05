%% SHOUTING KIDS WANT THEIR BALLOON
% this file serves as the main
% Initialization and plotting of the kids movement

close all
clear
clc

%% Model Initialization
kid.N = 10;                 % Number of kids        
kid.r = 0.35;               % radius of kids [m]
%balloon.N = 1;             % Number of balloons => later
params.roomWidth = 20;      % Size of the room in x-direction [m]
params.roomLength = 30;     % Size of the room in y-direction [m]

% random target position for each kid
kid.Destination = [randi([1 params.roomWidth-1], kid.N, 1), ...
                   randi([params.roomLength-10 params.roomLength], kid.N, 1)]; 

% random starting position for each kid
StartingPos = [randi([1 params.roomWidth-1], kid.N, 1), ... % all kids start in lower part
               randi([1 5], kid.N, 1)];    % dimension (kid.N x 2)

kid.Positions = [0 0];
while ~isequal(size(kid.Positions), size(StartingPos))   
    % Remove eventual repetitions from array; 'stable' to unselect sorting
    kid.Positions = unique(StartingPos, 'stable','rows');
    % insert new rows to reach number N
    if ~isequal(size(kid.Positions), size(StartingPos))
        l = length(StartingPos) - length(kid.Positions);
        kid.Positions = [kid.Positions; randi([1 10], l, 2)];
    end
end

% Assign a specific velocity to each kid between 0.5 and 2.2 m/s
kid.DesiredVel = 0.5 + (2.2 - 0.5) * rand(10,1);   % [m/s]
kid.Velocities = zeros(10,2);    % Only for initial step



% Plot initial positions
figure(12)
plot(kid.Positions(:,1), kid.Positions(:,2), 'ro'); % Red circles for kids
hold on;
axis equal;
plot(kid.Destination(:,1), kid.Destination(:,2), 'g*')
axis([1, params.roomWidth, 1, params.roomLength]);
title('Initial Positions');
legend('Current Position', 'Target Position','Location', 'best');
xlabel('Width');
ylabel('Length');
hold off;

% Solution with ode45, takes ages and only final plot can be seen, not the
% movement
% since i changed the desired velocity the plots seem random and the kids
% don't move towards their destination...
%[kid.Positions, kid.Velocities] = SFM1(kid, params);

%% Update and plot position according to SFM
% Number of simulation steps
params.numSteps = 1000;     % 
% Initialize matrices to store past positions
% pastKidPositions = zeros(kid.N, 2, params.numSteps);
pastKidPosToPlot = kid.Positions;   % will be enlarged by vertcat in loop

logFile = fopen('debugging.txt','w');

for step = 1:params.numSteps
    % Update animal positions based on flocking algorithm
    [kid.Positions, kid.Velocities] = SFM(kid, params);%, logFile);
    
    % % Store the current positions in the past positions matrices
    % pastKidPositions(:,:,step) = kid.Positions;
    % 
    
    % Plot the past animal positions as grey dots
    % Flatten the pastAnimalPositions matrix for plotting
    figure(12)
    pastKidPosToPlot = vertcat(pastKidPosToPlot, kid.Positions);
    plot(pastKidPosToPlot(:,1), pastKidPosToPlot(:,2), '.', 'Color', [0.8 0.8 0.8]);
    hold on;
     
    % Plot the current kid positions as red circles
    plot(kid.Positions(:,1), kid.Positions(:,2), 'ro');
    hold on;
    
    % Plot settings
    axis equal;
    title('Shouting kids want their balloons');
    axis([1, params.roomWidth, 1, params.roomLength]);    
    plot(kid.Destination(:,1), kid.Destination(:,2), 'g*')
    legend('Path taken', 'Current positions', 'Balloon', 'Location', 'best');
    hold off; % Clear the current plot for the next iteration

end

fclose(logFile);



