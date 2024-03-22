%% SHOUTING KIDS WANT THEIR BALLOON
%  Course:
%  Professor:
%  Academic Year:
%  Student:

clc
clear all
close all

%% Parameter initialization

% Number of Kids
numKid = 12;
% Number of Balloon
numBal = 12;

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 200;  % [cm]  2;    % [m] 
Room.Height = MaxNum * 150; % [cm]  1.5; 


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

% just for now for the first step, otherwise it's all zero
% KidArray.Destinations = BalloonArray.Positions;

% Second array only for the optimization
KidArrSFM = KidArray;
BalArrSFM = BalloonArray;

% check again what i actually meant with those...
params.Case = 1;
params.t = 1.5;
run = 1;
print_flag = 1;

while run
    
% TO DO:
    % change back to [m], not [cm]
    % tidy up the scripts, already better, do some more!
    % implement the cases !!
    % add the numbers from figure 1 also to figure 5
    % include the destinations to KidArray again for plot 8


    [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params);


    % Save old positions
    oldPos = KidArray.Positions;
    oldVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    % Update the final KidArray with new values of SFM
    for i = 1:KidArrSFM.N       
        % Update Position and actual velocity
        KidArray.Positions(KidArrSFM.ID(i),:) = KidArrSFM.Positions(i,:);
        KidArray.ActualVel(KidArrSFM.ID(i),:) = KidArrSFM.ActualVel(i,:);  
    end
    % Estimate total path length and time it took to reach goal
    deltaDistance = diag(pdist2(KidArray.Positions, oldPos));
    KidArray.PathLength = KidArray.PathLength + deltaDistance;
    newVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    AvgVel = (oldVel + newVel)/2;
    KidArray.TravelTime = KidArray.TravelTime + deltaDistance./AvgVel;


    
    % !! Units !!
    arrived = all(abs(KidArrSFM.Positions - KidArrSFM.Destinations)<[20 20],2);
    if any(arrived)
        % Extract ID of kids that reached their balloon
        ID_KidsArrived = KidArrSFM.ID(arrived);

        % Print output to command window
        if print_flag
            fprintf("The following kids have reached their balloon: ");
            print_flag = 0;
        end
        fprintf("%d ", ID_KidsArrived');
        
        
        % Exclude kids that have arrived from the next optimization step 
        fields = fieldnames(KidArrSFM);
        r = find(ismember(KidArrSFM.ID,ID_KidsArrived));
        for i = 6:numel(fields)
        % start from 6 bc the first five are N, radius, PathLength,
        % TravelTime, and InitPos --> not of interest in KidArrSFM
            % Get the matrix from the current field
            originalMatrix = KidArrSFM.(fields{i});
            
            % Delete the arrived kids from all the matrices
            updatedMatrix = originalMatrix;
            updatedMatrix(r, :) = [];
                        
            % Update the struct with the modified matrix
            KidArrSFM.(fields{i}) = updatedMatrix;
        end
        

        % Do the same for the balloons (esp. for their position)
            % --> find shorter more elegant solution later!!
        fields1 = fieldnames(BalArrSFM);
        r = find(ismember(BalArrSFM.ID,ID_KidsArrived));
        for i = 3:numel(fields1)
        % start from 3 bc the first two are N and edge
            % Get the matrix from the current field
            originalMatrix1 = BalArrSFM.(fields1{i});
            
            % Delete the arrived kids from all the matrices
            updatedMatrix1 = originalMatrix1;
            updatedMatrix1(r, :) = [];

            % Update the struct with the modified matrix
            BalArrSFM.(fields1{i}) = updatedMatrix1;
        end
           

        % Update N in KidArrSFM
        KidArrSFM.N = KidArrSFM.N - length(ID_KidsArrived);
        BalArrSFM.N = BalArrSFM.N - length(ID_KidsArrived);

        % Exit loop
        if KidArrSFM.N == 0
            fprintf("\n");
            run = 0;
        end

    end

end


%% Evaluation of the results
% Chosen metrics to verify the validity of the social experiment
%   - travelled path length
%   - overall travel time 
%   - 

PlotMetrics(KidArray);


