
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
numKid = 16;
% Number of Balloon
numBal = 16;

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]   



% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room.Width, Room.Height);

% Number of sensor
% Sensor.Num = 7;
% Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

%% Distributed Least Squares Algorithm 

min_SensorNum = 5;
max_SensorNum = 15;

Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);

KidArray.EstimatedPos = EstimatePosition(KidArray, Sensor, Room);
%BalloonArray.EstimatedPos = EstimatePosition(BalloonArray, Sensor, Room);



%% Kids' movement

% Use copy of arrays for the optimization
KidArrSFM = KidArray;
BalArrSFM = BalloonArray;

% check again what i actually meant with those...
params.Case = 1;
params.Subcase = 1;
params.t = 1.5;
run = 1;
print_flag = 1;

while run
    
%% TO DO:
    % tidy up the scripts, already better, do some more!
    % implement the cases !!
    % include the destinations to KidArray again for plot 8
    % Make sure estimatedPos is inside the while and gets updated at every
    % step
    % play with parameters (sim time) a bit


    %% Call the Social Force Model
    [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params);

    %% Estimate positions of kids with sensors
    % if params.Case == 1 && params.Subcase == 2
    %     % in these cases we need estimated positions
    %     KidArrSFM.EstimatedPos = EstimatePosition(KidArrSFM, Sensor, Room);
    % end

    %% Save previous positions
    oldPos = KidArray.ActualPos;
    oldVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    % Update the final KidArray with new values of SFM
    for i = 1:KidArrSFM.N       
        % Update Position and actual velocity
        KidArray.ActualPos(KidArrSFM.ID(i),:) = KidArrSFM.ActualPos(i,:);
        KidArray.ActualVel(KidArrSFM.ID(i),:) = KidArrSFM.ActualVel(i,:);  
    end
    % Estimate total path length and time it took to reach goal
    deltaDistance = diag(pdist2(KidArray.ActualPos, oldPos));
    KidArray.PathLength = KidArray.PathLength + deltaDistance;
    newVel = sqrt(sum(KidArray.ActualVel.^2, 2));
    AvgVel = (oldVel + newVel)/2;
    KidArray.TravelTime = KidArray.TravelTime + deltaDistance./AvgVel;

  
    %% Check if any kid has reached a balloon
    arrived = all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[0.2 0.2],2);
    if any(arrived)
        % Extract ID of kids that reached their balloon
        ID_KidsArrived = KidArrSFM.ID(arrived);


        %% Print output to command window
        % Discern between the cases
        if print_flag
            fprintf("The following kids have reached their balloon: ");
            print_flag = 0;
        end
        fprintf("%d ", ID_KidsArrived');

        
        %% Exclude kids that have arrived from the next optimization step 
        fields = fieldnames(KidArrSFM);
        KidArrSFM.ID_arr = find(ismember(KidArrSFM.ID,ID_KidsArrived));
        indx_ID = find(strcmp(fields, 'ID'));
        for i = indx_ID:numel(fields)
        % start from ID bc all fields before that are of no interest in KidArrSFM
            % Get the matrix from the current field
            originalMatrix = KidArrSFM.(fields{i});
            
            % Delete the arrived kids from all the matrices
            updatedMatrix = originalMatrix;
            updatedMatrix(KidArrSFM.ID_arr, :) = [];
                        
            % Update the struct with the modified matrix
            KidArrSFM.(fields{i}) = updatedMatrix;
        end
        
        % Do the same for the balloons
        fields1 = fieldnames(BalArrSFM);
        BalArrSFM.ID_arr = find(ismember(BalArrSFM.ID,ID_KidsArrived));
        indx_ID1 = find(strcmp(fields1, 'ID'));
        for i = indx_ID1:numel(fields1)
        % start from ID
            % Get the matrix from the current field
            originalMatrix1 = BalArrSFM.(fields1{i});
            
            % Delete the arrived kids from all the matrices
            updatedMatrix1 = originalMatrix1;
            updatedMatrix1(BalArrSFM.ID_arr, :) = [];

            % Update the struct with the modified matrix
            BalArrSFM.(fields1{i}) = updatedMatrix1;
        end
           

        % Update N in KidArrSFM
        KidArrSFM.N = KidArrSFM.N - length(ID_KidsArrived);
        BalArrSFM.N = BalArrSFM.N - length(ID_KidsArrived);

        %% Condition to exit loop
        if KidArrSFM.N == 0
            fprintf("\n");
            disp("-------------------------");
            run = 0;
        end

    end

end

%% Evaluation of the results
% Chosen metrics to verify the validity of the social experiment
%   - travelled path length
%   - overall travel time 
%   - 

% PlotMetrics(KidArray);


