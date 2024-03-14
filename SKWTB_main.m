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

%MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 200;
Room.Height = MaxNum * 150;

% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room.Width, Room.Height);

% Number of sensor
Sensor.Num = 7;

Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

%% Distributed Least Squares Algorithm 

max_SensorNum = 15;
min_SensorNum = 5;

Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);













