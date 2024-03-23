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

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 1.5;  % [m]   200;  % [cm]                      
Room.Height = MaxNum ;      % [m]   150;  % [cm]   


% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room.Width, Room.Height);

% Number of sensor
Sensor.Num = 7;
Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

%% Distributed Least Squares Algorithm 

max_SensorNum = 15;
min_SensorNum = 5;

Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);


%% Kids' movement

% just for now
KidArray.Destinations = BalloonArray.Positions;


% Second array only for the optimization
KidArrSFM = KidArray;
BalArrSFM = BalloonArray;

% check again what i actually meant with those...
params.Case = 2;
params.t = 1.5;
run = 1;
r_old = 0;  % initially no kid has reached destination 

while run
    
% to do:
    % change back to meters, not cm
    % implement measurement of path length as a metric
    % tidy up the scripts
    % join them with jacopo's in git
    % 


    [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params);


    % save old positions
    oldPos = KidArray.Positions;
    oldVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    % Update the final KidArray with new values of SFM
    for i = 1:KidArrSFM.N       
        % Update Position and actual velocity
        KidArray.Positions(KidArrSFM.ID(i),:) = KidArrSFM.Positions(i,:);
        KidArray.ActualVel(KidArrSFM.ID(i),:) = KidArrSFM.ActualVel(i,:);
        % (now the positions are updated)         
    end
    % Estimate total path length and time it took to reach goal
    deltaDistance = diag(pdist2(KidArray.Positions, oldPos));
    KidArray.PathLength = KidArray.PathLength + deltaDistance;
    newVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    AvgVel = (oldVel + newVel)/2;
    KidArray.TravelTime = KidArray.TravelTime + deltaDistance/AvgVel;


    

    if any(all(abs(KidArray.Positions - KidArray.Destinations)<[20 20],2))
        [r, ~] = find(all(abs(KidArray.Positions - ...
                                KidArray.Destinations) < [20 20], 2));
        
        % Execution only when r has changed, r_old is updated after loop
        if length(r) > length(r_old)

            fprintf("The following kids have reached their balloon: ");
            fprintf("%d ", r');
            fprintf("\n");

            % Exclude kids that arrived from next optimization step 
                % make it more elegant with the ID's
            fields = fieldnames(KidArray);
            for i = 5:numel(fields)
            % start from 5 bc the first four are N, radius, PathLength,
            % and TravelTime --> not of interest in KidArrSFM
                % Get the matrix from the current field
                originalMatrix = KidArray.(fields{i});
                
                % Delete the arrived kids from the all matrices (rows r)
                updatedMatrix = originalMatrix;
                updatedMatrix(r, :) = [];
                
                % Update the struct with the modified matrix
                KidArrSFM.(fields{i}) = updatedMatrix;
            end
    
            % Do the same for the balloons (esp. for their position)
                % --> find shorter more elegant solution later!!
            fields1 = fieldnames(BalloonArray);
            for i = 3:numel(fields1)
            % start from 3 bc the first two are N and edge
                % Get the matrix from the current field
                originalMatrix1 = BalloonArray.(fields1{i});
                
                % Delete the arrived kids from the all matrices (rows r)
                updatedMatrix1 = originalMatrix1;
                updatedMatrix1(r, :) = [];
                
                % Update the struct with the modified matrix
                BalArrSFM.(fields1{i}) = updatedMatrix1;
            end

            % Update N in KidArrSFM
            KidArrSFM.N = KidArray.N - length(r);
            BalArrSFM.N = BalloonArray.N - length(r);

        end

        % Remember current number of rows
        r_old = length(r);

        % Exit loop
        if length(r) == KidArray.N
            run = 0;
        end

    end




end


