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
numRuns = 5;

Case = 1;
subcase = 1;
distrpattern = 3; 

CasesNum = 7;

numkids = 8;
numballs = 8;
trajectoryflag = 1;
videoflag = 1;

% Preallocate a cell array to store results
% total_time = zeros(numRuns, 1);
% travel_time= zeros(numRuns, numkids);
% path_length = zeros(numRuns, numkids);
% avg_vel = zeros(numRuns, numkids);
% des_vel = zeros(numRuns, numkids);

total_time = zeros(numRuns, 1, CasesNum);
travel_time= zeros(numRuns, numkids, CasesNum);
path_length = zeros(numRuns, numkids, CasesNum);
avg_vel = zeros(numRuns, numkids, CasesNum);
des_vel = zeros(numRuns, numkids, CasesNum);

% Loop to run script multiple times
% for runIdx = 1:numRuns
% 
%     % Run the script
%     res  = runrun(Case, subcase, distrpattern, numkids, numballs, trajectoryflag, videoflag);
% 
%     % Save the result to the cell array
%     travel_time(runIdx,:) = res.TravelTime';
%     total_time(runIdx) = res.TotalTime;
%     path_length(runIdx,:) = res.PathLength;
%     avg_vel(runIdx,:) = res.AvgVel;
%     des_vel(runIdx,:) = res.DesVel;
% 
% end

% Loop to run script multiple times and change the case

for caseInd = 1:CasesNum

    if caseInd == 1
        Case = 1;
        subcase = 1;
    elseif caseInd == 2
        Case = 1;
        subcase = 2;
    elseif caseInd == 3
        Case = 2;
        subcase = 1;
    elseif caseInd == 4
        Case = 2;
        subcase = 2;
    elseif caseInd == 5
        Case = 3;
        subcase = 1;
    elseif caseInd == 6
        Case = 3;
        subcase = 2;
    elseif caseInd == 7
        Case = 4;
        subcase = 1;
    end

        for runIdx = 1:numRuns
        
        % Run the script
        res  = runrun(Case, subcase, distrpattern, numkids, numballs, trajectoryflag, videoflag);
    
        % Save the result to the cell array
        travel_time(runIdx,:, caseInd) = res.TravelTime';
        total_time(runIdx, 1, caseInd) = res.TotalTime;
        path_length(runIdx,:, caseInd) = res.PathLength;
        avg_vel(runIdx,:, caseInd) = res.AvgVel;
        des_vel(runIdx,:, caseInd) = res.DesVel;
        
        end
end

% Save all results in one file
save('./warehouse/TotalTime.mat', 'total_time');
save('./warehouse/TravelTime.mat', 'travel_time');
save('./warehouse/Pathlength.mat', 'path_length');
save('./warehouse/AverageVel.mat', 'avg_vel');
save('./warehouse/DesiredVel', 'des_vel');


i = 1;
j = 1;

TravelTimeEx = "./warehouse/Case" + num2str(Case) + num2str(subcase) + ...
               "_" + num2str(distrpattern) + "_TravelTime_" + num2str(i) + ".xls";

PathLengthEx = "./warehouse/Case" + num2str(Case) + num2str(subcase) + ...
               "_" + num2str(distrpattern) + "_PathLength_" + num2str(j) + ".xls";

while isfile(TravelTimeEx)
        i = i+1;
        TravelTimeEx = "./warehouse/Case" + num2str(Case) + num2str(subcase) + ...
               "_" + num2str(distrpattern) + "_TravelTime_" + num2str(i) + ".xls";
end

while isfile(PathLengthEx)
        j = j+1;
        PathLengthEx = "./warehouse/Case" + num2str(Case) + num2str(subcase) + ...
               "_" + num2str(distrpattern) + "_PathLength_" + num2str(j) + ".xls";
end

writematrix(travel_time, TravelTimeEx)
writematrix(path_length, PathLengthEx)

disp('Done.')


