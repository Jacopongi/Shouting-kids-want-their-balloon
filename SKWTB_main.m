
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
numKid = 7;
% Number of Balloon
numBal = 7;

% MaxNum = max(numKid, numBal);
MaxNum = numKid + numBal;

% Room features
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]   



% Random positioning of kids and balloons inside the room
[KidArray, BalloonArray] = ...
    distributeKidBalloon(numKid, numBal, Room.Width, Room.Height, 3);

% Number of sensor
% Sensor.Num = 7;
% Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

%% Distributed Least Squares Algorithm 

min_SensorNum = 5;
max_SensorNum = 15;

Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);


% First estimation of the initial starting position
KidArray.EstimatedPos = EstimatePosition(KidArray, Sensor, Room);
%BalloonArray.EstimatedPos = EstimatePosition(BalloonArray, Sensor, Room);



%% Simulation settings

% Use copy of arrays for the optimization
KidArrSFM = KidArray;
BalArrSFM = BalloonArray;

% Parameters & flags
params.Case = 1;
params.Subcase = 2;
params.t = 1.5;
params.flagForce = 0;   % de-/activate the repulsive force of non-targeted balloons
run = 1;
print_flag = 1;


%% Main section


while run

    %% Call the Social Force Model
    [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params);

    %% Estimate positions of kids with sensors
    if (params.Case == 1 && params.Subcase == 2) || (params.Case == 2) || ...
            (params.Case == 3)
        % The updated positions that we obtain from the SFM are the new
        % actual positions after we've shifted the initial
        % in these cases we need to estimate t
        KidArrSFM.EstimatedPos = EstimatePosition(KidArrSFM, Sensor, Room);
    end

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
    if (params.Case == 1) || (params.Case == 2)
        touch = KidArray.Radius + BalloonArray.Edge/2;
        arrived = all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[touch touch],2);
    elseif (params.Case == 3)
        touch = KidArray.Radius + BalloonArray.Edge;    % choose a bit bigger here to try out
        % arrived = all(abs(KidArrSFM.ActualPos - BalArrSFM.ActualPos)<[touch touch],2);

        % this index belongs to the balloon at which the kid arrived
        [dist,indx] = min(pdist2(KidArrSFM.ActualPos,BalArrSFM.ActualPos),[],2);
        arrived = find(dist<touch);
    end

    if any(arrived)
        % Extract ID of kids that reached their balloon
        ID_KidsArrived = KidArrSFM.ID(arrived);

       
        
        %% Exclude kids that have arrived from the next optimization step 
        % Distinguish between cases!
      
        if (params.Case == 1)
            
            % Print output to command window
            if print_flag
                fprintf("Kids that have reached their balloon: ");
                print_flag = 0;
            end
            fprintf("%d ", ID_KidsArrived');


            % Reveal color and ID of the balloon      
            for i = 1:length(ID_KidsArrived)
                x_min_b = BalArrSFM.InitPos(ID_KidsArrived(i),1) - 0.5*BalArrSFM.Edge; 
                y_min_b = BalArrSFM.InitPos(ID_KidsArrived(i),2) - 0.5*BalArrSFM.Edge;
                x_max_b = BalArrSFM.Edge;
                y_max_b = BalArrSFM.Edge;
                BalArrSFM.squarefig(ID_KidsArrived(i)) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
                    'FaceColor', KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_KidsArrived(i)))),:));       
                BalArrSFM.plotBalID(ID_KidsArrived(i)) = text(BalArrSFM.InitPos(ID_KidsArrived(i),1), ...
                    BalArrSFM.InitPos(ID_KidsArrived(i),2), num2str(ID_KidsArrived(i)), ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalArrSFM.Edge*10);
            end



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
        end

        %% CASE 2
        if (params.Case == 2)
            % Check if the kids arrived at their correct balloon (same ID)
            % extract ID's of the closest balloons --> Destinations
                   
            MemoPosReceived = zeros(length(ID_KidsArrived),1); % length of initial NKidArrSFM.ID_arr

            for h = 1:length(ID_KidsArrived)

                BalPos = BalArrSFM.ActualPos;

                % get row 
                rowKidID = find(ismember(KidArrSFM.ID, ID_KidsArrived(h), 'rows'));
                KidTarget = KidArrSFM.Destinations(rowKidID,:);                
                
                % rowBalPos = find(ismember(BalPos, KidDest, 'rows'));
                % ID_Bal2Check = BalArrSFM.ID(rowBalPos);
                   % !!!! here allowed bc same deletion 
             
                ID_Bal2Check = find(ismember(BalArrSFM.InitPos, KidTarget, 'rows'));

                % Reveal color and ID of the balloon  
                if ismember(ID_Bal2Check, KidArrSFM.ID)
                    x_min_b = BalArrSFM.InitPos(ID_Bal2Check,1) - 0.5*BalArrSFM.Edge; 
                    y_min_b = BalArrSFM.InitPos(ID_Bal2Check,2) - 0.5*BalArrSFM.Edge;
                    x_max_b = BalArrSFM.Edge;
                    y_max_b = BalArrSFM.Edge;
                    BalArrSFM.squarefig(ID_Bal2Check) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
                        'FaceColor', KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_Bal2Check))),:));       
                    BalArrSFM.plotBalID(ID_Bal2Check) = text(BalArrSFM.InitPos(ID_Bal2Check,1), ...
                        BalArrSFM.InitPos(ID_Bal2Check,2), num2str(ID_Bal2Check), ...
                        'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalArrSFM.Edge*10);
                end


                if (ID_KidsArrived(h) == ID_Bal2Check)

                    if print_flag
                        fprintf("Kids that have reached their balloon: ");
                        print_flag = 0;
                    end
                    fprintf("%d ", ID_KidsArrived(h));




                    % Match found --> removal process for both kid X and
                    % its balloon => same as for case 1
                    fields = fieldnames(KidArrSFM);
                    KidArrSFM.ID_arr = find(ismember(KidArrSFM.ID,ID_KidsArrived(h)));
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
                    BalArrSFM.ID_arr = find(ismember(BalArrSFM.ID,ID_KidsArrived(h)));
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
    
                    KidArrSFM.N = KidArrSFM.N - 1;

                    % Put it also in the BalVisited matrix
                    for i = 1:length(KidArrSFM.BalVisited)
                        col = find(KidArrSFM.BalVisited(i, :) == 0, 1);
                        row = find(ismember(KidArrSFM.ID,i));
                        if ~isempty(row)
                            if ~ismember(ID_Bal2Check,KidArrSFM.BalVisited(KidArrSFM.ID(row),:))
                                KidArrSFM.BalVisited(i, col) = ID_Bal2Check;
                            end
                        end
                        
                    end
            

                else 
                    % (ID_KidsArrived(h) ~= ID_Bal2Check), no match
                    if (params.Subcase == 1)
                        % let the kid proceed to the next closest balloon
                        %   - balloon needs to remain untouched
                        %   - kid needs to stay in the game
                        %   - BUT: Current destination needs to be excluded
                        %          from the comparison done in SFM2 --> maybe
                        %          move it to here??
    
                        % create a flag to mark wrong match between kid and balloon
                        % params.flagWrongBal = [params.flagWrongBal, ID_KidsArrived(h)];
    
                        % Save already visited balloons 
                        col = find(KidArrSFM.BalVisited(ID_KidsArrived(h), :) == 0, 1);
                        if isempty(ID_Bal2Check)
                            ID_Bal2Check = 0;
                        end
                        KidArrSFM.BalVisited(ID_KidsArrived(h), col) = ID_Bal2Check;
                    
                    elseif (params.Subcase == 2)
                        % Send message to the other kids => assign a fixed
                        % destination for the respective kid and put the
                        % respective balloon as "already visited" for all
                        % others

                        for i = 1:length(KidArrSFM.BalVisited)
                            col = find(KidArrSFM.BalVisited(i, :) == 0, 1);
                            if i == ID_Bal2Check
                                % "send message" / assign new final destination
                                % make use of memory matrix and set them
                                % only after all ID_KidsArrived (index h)
                                % have been run through
                                MemoPosReceived(h) = i;
                                        % KidArrSFM.Destinations(find(ismember(KidArrSFM.ID,i)),:) = ...
                                        %    BalArrSFM.ActualPos(find(ismember(KidArrSFM.ID,i)),:);
                                % prevent overwriting later-on
                                KidArrSFM.FlagPosReceived(i) = 1;
                            else
                                if isempty(ID_Bal2Check)
                                    ID_Bal2Check = 0; % shouldn't be necessary anymore now that we take InitPos for ID_Bal2Check
                                end
                                row = find(ismember(KidArrSFM.ID,i));
                                if ~isempty(row)
                                    if ~ismember(ID_Bal2Check,KidArrSFM.BalVisited(KidArrSFM.ID(row),:))
                                        KidArrSFM.BalVisited(i, col) = ID_Bal2Check;
                                    end
                                end
                            end
                        end
                    end
                end % if (ID_KidsArrived(h) == ID_Bal2Check)
            end % for h = 1:length(ID_KidsArrived)

            % set the new destinations here
            row_of_ID = find(ismember(KidArrSFM.ID,MemoPosReceived));
            %KidArrSFM.ID(find(MemoPosReceived));
            KidArrSFM.Destinations(row_of_ID,:) = BalArrSFM.ActualPos(row_of_ID,:);

        end  

        %% CASE 3
        % In contrast to case 2, the sent message needs to include the
        % estimate of the kid's actual position at the time step when it
        % arrived at a balloon and NOT directly the balloon's position or
        % it's own actual position.
        % Also, not all other kids should be informed but only the one to
        % whom the balloon belongs. If another kid stops by the same
        % balloon it also sends his estimated position so with time the
        % actual kid gets an increasingly better estimate of its balloon's
        % location.
        % All kids continue to wander around randomly until they've found
        % their own balloon.
       
        if (params.Case == 3)
            % 
            for h = 1:length(ID_KidsArrived)

                % get row of arrived kid
                rKidID_arr = find(ismember(KidArrSFM.ID, ID_KidsArrived(h), 'rows'));
                [~,ID_Bal2Check] = min(sum(abs(KidArrSFM.ActualPos(rKidID_arr,:) - ...
                                BalArrSFM.InitPos),2));
                
                % Reveal color and ID of the balloon  
                if ismember(ID_Bal2Check, KidArrSFM.ID)
                    x_min_b = BalArrSFM.InitPos(ID_Bal2Check,1) - 0.5*BalArrSFM.Edge; 
                    y_min_b = BalArrSFM.InitPos(ID_Bal2Check,2) - 0.5*BalArrSFM.Edge;
                    x_max_b = BalArrSFM.Edge;
                    y_max_b = BalArrSFM.Edge;
                    BalArrSFM.squarefig(ID_Bal2Check) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
                        'FaceColor', KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_Bal2Check))),:));       
                    BalArrSFM.plotBalID(ID_Bal2Check) = text(BalArrSFM.InitPos(ID_Bal2Check,1), ...
                        BalArrSFM.InitPos(ID_Bal2Check,2), num2str(ID_Bal2Check), ...
                        'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalArrSFM.Edge*10);
                end

                if (ID_KidsArrived(h) == ID_Bal2Check)

                    if print_flag
                        fprintf("Kids that have reached their balloon: ");
                        print_flag = 0;
                    end
                    fprintf("%d ", ID_KidsArrived(h));


                    % Match found --> removal process for both kid X and
                    % its balloon => same as for case 1
                    fields = fieldnames(KidArrSFM);
                    KidArrSFM.ID_arr = find(ismember(KidArrSFM.ID,ID_KidsArrived(h)));
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
                    BalArrSFM.ID_arr = find(ismember(BalArrSFM.ID,ID_KidsArrived(h)));
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
    
                    KidArrSFM.N = KidArrSFM.N - 1;

                    % continue ?? => no, we have no BalVisited in this case

                else
                    % not a match => send message to the respective kid
                    % giving them their new destination
                    [~,rKid_msg] = ismember(ID_Bal2Check, KidArrSFM.ID);
                    if rKid_msg 
                        % if two kids arrive simultaneously at the same
                        % balloon enter loop only one otherwise error
                        KidArrSFM.Destinations(rKid_msg,:) = ...
                            KidArrSFM.EstimatedPos(rKidID_arr,:);
                    end

                    % FlagPosReceived to prevent overwriting of next
                    % destination
                    KidArrSFM.FlagPosReceived(ID_Bal2Check) = 1;                   

                end

            end

        end

        % Update N in KidArrSFM
        if (params.Case == 1)
            KidArrSFM.N = KidArrSFM.N - length(ID_KidsArrived);
            BalArrSFM.N = BalArrSFM.N - length(ID_KidsArrived);
        
        end

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
%   - traveled path length
%   - overall travel time 
%   - 

% PlotMetrics(KidArray);


