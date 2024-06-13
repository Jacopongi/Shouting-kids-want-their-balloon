
%% SHOUTING KIDS WANT THEIR BALLOON
%  Course:        Intelligent Distributed Systems
%  Professor:     Fontanelli Daniele
%  Academic Year: 2023-2024
%  Students:      Endrizzi Jacopo  
%                 Pfluger Thade  
            

clc
clear
close all

%% Parameter initialization

% Number of Kids
numKid = 7;
% Number of Balloon
numBal = 7;

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]  

% Parameters & flags
params.Case = 4;
params.Subcase = 1;
params.t = 0.5;

params.plotTrajEst = 1; % if 0 plot actual position
params.NumSFMExec = 0;
params.flagForce = 1;   % de-/activate the repulsive force of non-targeted balloons
params.print_flag = 1;
params.occupancyMap = zeros(5*Room.Height, 5*Room.Width); % !rows are height
params.VideoFlag = 0;
params.frames = cell(1,1);

% 1=random, 2=grid-grid, 3=grid-arc, 4=circular
distrPattern = 3;



% Random or patterned positioning of kids and balloons inside the room
[KidArray, BalloonArray, params] = ...
            distributeKidBalloon(numKid, numBal, Room, distrPattern, params);

% Number of sensor
% Sensor.Num = 7;
% Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

%% Distributed Least Squares Algorithm 

min_SensorNum = 5;
max_SensorNum = 15;

Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);


%% Simulation settings

% Use one copy for the optimization and keep the other one for the metrics
KidArrSFM = KidArray;
BalArrSFM = BalloonArray; 



%% Main section
run = 1;

while run

    % Call the Social Force Model
    [KidArrSFM, BalArrSFM, params] = ...
                    SFM2(KidArrSFM, BalArrSFM, Sensor, Room, params);

    % Call CaseDistinction to process data according to correct case
    [KidArrSFM, BalArrSFM, params] = ...
                    CaseDistinction(KidArrSFM, BalArrSFM, Sensor, Room, params);
                

    % Condition to exit loop
    if KidArrSFM.N == 0
        fprintf("\n");
        disp("-------------------------");
        run = 0;
    end
end


%% Obtain a video from animation 
if params.VideoFlag
    % capture last frame and duplicate it to make end of video nicer
    figure(1)
    frame = getframe(gcf);
    for i = 1:10
        params.frames{end+1} = frame;  
    end

    % prepare filename to prevent overwriting of previous recordings
    i = 1;
    filename = "./figures/Case" + num2str(params.Case) + num2str(params.Subcase) + ...
               "_" + num2str(distrPattern) + "_" + num2str(i) + ".avi";
    while isfile(filename)
        i = i+1;
        filename = "./figures/Case" + num2str(params.Case) + num2str(params.Subcase) + ...
                   "_" + num2str(distrPattern) + "_" + num2str(i) + ".avi";
    end

    v = VideoWriter(filename);   
    v.FrameRate = 10;                 
    open(v);                        

    % Write captured frames to VideoWriter
    for i = 1:length(params.frames)
        writeVideo(v, params.frames{i});
    end
    close(v);
end

%% Evaluation of the results
% Chosen metrics to verify the validity of the social experiment
%   - traveled path length
%   - overall travel time 
%   - 

PlotMetrics(KidArrSFM, KidArray);


