
%% SHOUTING KIDS WANT THEIR BALLOON
%  Course:        Intelligent Distributed Systems
%  Professor:     Fontanelli Daniele
%  Academic Year: 2023-2024
%  Students:      Endrizzi Jacopo  
%                 Pfluger Thade  
            

clc
clear all
close all

%% Parameter initialization

% Number of Kids
numKid = 10;
% Number of Balloon
numBal = 10;

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]   

% Random or patterned positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room, 1);

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

% Parameters & flags
params.Case = 3;
params.Subcase = 2;
params.t = 1.5;
params.NumSFMExec = 0;
params.flagForce = 0;   % de-/activate the repulsive force of non-targeted balloons
params.print_flag = 1;
run = 1;


%% Main section
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

%% Evaluation of the results
% Chosen metrics to verify the validity of the social experiment
%   - traveled path length
%   - overall travel time 
%   - 

PlotMetrics(KidArrSFM, KidArray);


