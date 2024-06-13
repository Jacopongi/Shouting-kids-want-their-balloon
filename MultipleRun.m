clc
close all
clear all

% Summary: wrapper file to repeat several times the simulation with and without flocking. 
% Description: robots try to reach their target (specified by an identity number)
% When multiple robots come close, they start to move together (flocking).
% They try to avoid other obstacles. 
% If a robot is pretty near its target it stops.

% NOTE: there can be more robots than targets and more targets than robots.
% Simulation stops when all robots have reached their target,
% or when all targets are reached.

% NOTE: FlockingFlag parameter lets user choose if flocking behavior have to be activated.
% It is possible to run the simulation with just collision avoidance behavior.
% Set FlockingFlag = false, for this second case.

% Basic parameters
numRuns = 1;
num_robots = 10;
num_obstacles = 5;

% Preallocate a cell array to store results
total_time_cs = cell(numRuns, 1);
%robot_steps = cell(numRuns,1);
robot_steps_cs = zeros(numRuns, num_robots);

% Loop to run script multiple times
for runIdx = 1:numRuns

    % Input parameters for each run
    % num_robots = runIdx; 
    % num_obstacles = runIdx + 2; 
    
    % Run the script
    [tt_cs, rs_cs]  = FlockingRuns(true, num_robots, num_obstacles);

    % Save the result to the cell array
    total_time_cs{runIdx} = tt_cs;
    robot_steps_cs(runIdx,:) = rs_cs;
    
    % Save the result to a .mat file
    % saveFileName = sprintf('result_run_%d.mat', runIdx);
    % save(saveFileName, 'result');
end

% Save all results in one file
save('TotalTimeCs.mat', 'total_time_cs');
save('RobotStepsCs.mat', 'robot_steps_cs');

% Re-allocate cells to save new results
total_time_nc = cell(numRuns, 1);
robot_steps_nc = zeros(numRuns, num_robots);

% New runs with function changes
for runIdx = 1:numRuns

    % Input parameters for each run
    % num_robots = runIdx; 
    % num_obstacles = runIdx + 2; 
    
    % Run the script
    [tt_cs, rs_cs]  = FlockingRuns(false, num_robots, num_obstacles);

    % Save the result to the cell array
    total_time_cs{runIdx} = tt_cs;
    robot_steps_cs(runIdx,:) = rs_cs;
    
    % Save the result to a .mat file
    % saveFileName = sprintf('result_run_%d.mat', runIdx);
    % save(saveFileName, 'result');
end

% Save all results in one file
save('TotalTimeNC.mat', 'total_time_nc');
save('RobotStepsNC.mat', 'robot_steps_nc');


