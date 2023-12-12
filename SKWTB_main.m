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
RoomWidth = 3000;
RoomHeight = 4000;

% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, RoomWidth, RoomHeight);

% Number of sensor
SensorNum = 9;

SensorsPosition = distributeSensorsOnPerimeter(SensorNum, RoomWidth, RoomHeight);













