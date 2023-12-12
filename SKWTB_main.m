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
RoomWidth = 4000;
RoomHeight = 4000;

% Dimensions 
radius = 30;
edge = 40;

% Random starting positions 
positions_Kid = rand(numKid,2);
positions_Bal = rand(numBal,2);

positions_Kid(:,1) = 0.1*RoomWidth + positions_Kid(:,1) * 0.8*RoomWidth;
positions_Kid(:,2) = 0.1*RoomHeight + positions_Kid(:,2) * 0.8*RoomHeight;

positions_Bal(:,1) = 0.1*RoomWidth + positions_Bal(:,1) * 0.8*RoomWidth;
positions_Bal(:,2) = 0.1*RoomHeight + positions_Bal(:,2) * 0.8*RoomHeight;

% Check distance between kids
dist_KidKid = 0;
dist_BalBal = 0;
dist_KidBal = 0;
while( (dist_KidKid == 0) || (dist_BalBal == 0) || (dist_KidBal == 0) )
        exit_for = 0;
        for i = 1:numKid
            for j = 1:numKid
                if i==j
                    continue;
                end 
                if ( sqrt( (positions_Kid(i,1) - positions_Kid(j,1)).^2 + (positions_Kid(i,2) - positions_Kid(j,2)).^2) < 2*radius) 
                    positions_Kid(i,:) = rand(2,1);
                    positions_Kid(i,1) = 0.1*RoomWidth + positions_Kid(i,1) * 0.8*RoomWidth;
                    positions_Kid(i,2) = 0.1*RoomHeight + positions_Kid(i,2) * 0.8*RoomHeight;
                    dist_KidKid = 0;
                    break;
                end
            end
            if (exit_for == 1)
                dist_KidKid = 0;
                break
            else 
                dist_KidKid = 1;
            end
        end
        
        % Check distance between balloons
        exit_for = 0;
        for i = 1:numBal
            for j = 1:numBal
                if i==j
                    continue;
                end 
                if ( sqrt( (positions_Bal(i,1) - positions_Bal(j,1)).^2 + (positions_Bal(i,2) - positions_Bal(j,2)).^2) < 2*edge) 
                    positions_Bal(i,:) = rand(2,1);
                    positions_Bal(i,1) = 0.1*RoomWidth + positions_Bal(i,1) * 0.8*RoomWidth;
                    positions_Bal(i,2) = 0.1*RoomHeight + positions_Bal(i,2) * 0.8*RoomHeight;
                    exit_for = 1;
                    break;
                end
            end
            if (exit_for == 1)
                dist_BalBal = 0;
                break
            else 
                dist_BalBal = 1;
            end
        end
        
        % Check distance between kids and balloons
        exit_for = 0;
        for i = 1:numKid
            for j = 1:numBal
                if ( sqrt( (positions_Kid(i,1) - positions_Bal(j,1)).^2 + (positions_Kid(i,2) - positions_Bal(j,2)).^2) < edge + radius) 
                    positions_Kid(i,:) = rand(2,1);
                    positions_Kid(i,1) = 0.1*RoomWidth + positions_Kid(i,1) * 0.8*RoomWidth;
                    positions_Kid(i,2) = 0.1*RoomHeight + positions_Kid(i,2) * 0.8*RoomHeight;
                    positions_Bal(j,:) = rand(2,1);
                    positions_Bal(j,1) = 0.1*RoomWidth + positions_Bal(i,1) * 0.8*RoomWidth;
                    positions_Bal(j,2) = 0.1*RoomHeight + positions_Bal(i,2) * 0.8*RoomHeight;
                    exit_for = 1;
                    break;
                end
            end
            if (exit_for == 1)
                dist_KidBal = 0;
                break
            else 
                dist_KidBal = 1;
            end
        end
end


% Random x and y velocities
max_velocity = 2.5;
velocities_Kid = rand(numKid,2).* max_velocity;

% Structure Arrays

for index = 1:numKid
    KidArray(index) = MakeKid(positions_Kid(index,1),positions_Kid(index,2), ...
                      velocities_Kid(index,1),velocities_Kid(index,2), ...
                      radius, index);
end

for index = 1:numBal
    BalloonArray(index) = MakeBalloon(positions_Bal(index,1),positions_Bal(index,2), ...
                          edge, index);
end

% Figure 
figure(1)
axis equal
axis([0 RoomWidth 0 RoomHeight])
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Kids in the room','FontSize',14)

circlefig = zeros(1,numKid);
color_Kid = rand(numKid,3);
for i = 1:numKid
    x_min = KidArray(i).x - KidArray(i).r;
    y_min = KidArray(i).y - KidArray(i).r;
    radius_cur = KidArray(i).r;
    circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],'Curvature',[1 1], 'FaceColor',color_Kid(i,:));
end

squarefig = zeros(1,numBal);
color_Bal = rand(numBal, 3);
for i = 1:numKid
    color_Bal(i,:) = color_Kid(i,:);
end
for i = 1:numBal
    x_min_b = BalloonArray(i).x - 0.5*BalloonArray(i).edge; 
    y_min_b = BalloonArray(i).y - 0.5*BalloonArray(i).edge;
    x_max_b = BalloonArray(i).edge;
    y_max_b = BalloonArray(i).edge;
    squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], 'FaceColor',color_Bal(i,:));
    BalloonArray(i);        
end


% Number of sensor
SensorNum = 17;

SensorsPosition = distributeSensorsOnPerimeter(SensorNum, RoomWidth, RoomHeight);









