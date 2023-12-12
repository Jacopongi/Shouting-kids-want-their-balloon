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
Room.Width = 3000;
Room.Height = 4000;

% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room);

% Number of sensor
SensorNum = 9;

SensorsPosition = distributeSensorsOnPerimeter(SensorNum, Room);



%% Social Force Model
% Interaction of the kids on their way to teh balloon based on social force
% model algorithm

[Positions, Velocities] = SFM1(KidArray, BalloonArray, Room);






