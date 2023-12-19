%% SHOUNTING KIDS WANT THEIR BALLOON
%  Course:
%  Professor:
%  Academic Year:
%  Student:

clc
clear all
close all

%% Parameter initialization

% Number of Kids
numKid = 10;
% Number of Balloon
numBal = 10;

% Room features
Room.Width = 30;
Room.Height = 40;

% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room);

% Number of sensor
SensorNum = 9;

SensorsPosition = distributeSensorsOnPerimeter(SensorNum, Room);



%% Social Force Model
% Interaction of the kids on their way to the balloon based on social force
% model algorithm

%params.logFile = fopen('debugging.txt','w');
run = 1;

params.t = 3;  % Simulation time
params.Case = 2;    % 1: Each kid runs to the closest balloon
                    % 2: Each kid knows which is their balloon   
                    % 3: 
                    % ...

     
while run
    
    [KidArray] = SFM1(KidArray, BalloonArray, Room, params);

    if any(all(abs(KidArray.Positions - KidArray.Destinations)<[0.2 0.2],2))
        [r, ~] = find(all(abs(KidArray.Positions - ...
                                KidArray.Destinations) < [0.2, 0.2], 2));
        fprintf("I'm kid %d and I reached a balloon!!\n", r);
        %return
    end

end

%fclose(params.logFile);

% hier brauch ma a schleife, de immer wieder des sfm aufruaft und
% zwischendrin zeit für kommunikation losst. Des werd dann eher wia des
% spui wo d'Musi lafft und auf pause miassns stehbleim. Des ganze dann mit
% dem logfile, des uns de ausgetauschten Nachrichten speichert. in SFM1
% brauch ma dann einige flags:
% case of communication between robots when there's no sensor




