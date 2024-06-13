function [KidArrSFM, BalArrSFM, params] = ...
                    CaseDistinction(KidArrSFM, BalArrSFM, Sensor, Room, params)
% In this function all the initial conditions and destinations are prepared
% to then be fed into the social force model function. For detailed
% description of the single cases refer to the section below.

%{
1)  With sensors, positions of kids and corresponding balloons are known.
	1.1 Use absolute positions as reference.
	1.2 Use estimated positions (with sensors) as case of study.
    Testing with different sensor number to verify correct functioning of "ChooseSensorNumber" function. 

2)  With sensors, positions of kids and balloons are known. 
    2.1 Kids are guided to closest balloon. Once reached, they verify if it belongs to them.
        If yes, they stop. Otherwise, they proceed to the next closest one.
    2.2 Alternative: they send a message with the balloon number so the
        corresponding kid is guided towards it.
	    All other kids are pushed away from that balloon (Repulsive force SFM).

3)  With sensors, known positions of kids only.
    Kids wander around. Once they find a balloon, they verify if it belongs to them.
    if true, they stop. Otherwise, they send a message with their estimated position. Corresponding kid is called to reach that position. 
    Spiral movement from estimated position if the balloon can't be seen from that position.
    Other kids passing near the balloon send messages to improve the estimation.  
    Alternative: they send a message with the balloon number and the corresponding kid is guided to it.


4)  Occupancy map with flocking (future work idea)
    For now hardcoded but could be improved with another consensus.

%}


%% Find out which function made the call
stack = dbstack;
callerFunction = stack(2).name;


if params.NumSFMExec == 1
    % First estimation of the initial starting position
    KidArrSFM.EstimatedPos = EstimatePosition(KidArrSFM, Sensor, Room, false);
end


if callerFunction == "SFM2"
    %% Case 1
    if params.Case == 1
        % Every kid knows which balloon is theirs and runs towards it
        KidArrSFM.Destinations = BalArrSFM.ActualPos;
        if params.Subcase == 1
            % Combine initial positions and velocities into a single vector
            % Use absolute positions as reference
            params.WithEstimatedPos = 0;
            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.ActualPos(:)];
        elseif params.Subcase == 2
            % Use estimated positions     
            params.WithEstimatedPos = 1;
            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
        end
    
    
    
    %% Case 2
    elseif params.Case == 2 
        params.WithEstimatedPos = 1;
        if (params.Subcase == 1) || (params.Subcase == 2 && params.NumSFMExec == 1)
    
          % MAKE THIS SECTION MORE UNDERSTANDABLE, IT SEEMS TO WORK BUT IT'S A
          % MESS TO UNDERSTAND WITH ALL THE NESTED COMMANDS !
    
            % every kid runs to the nearest balloon
            distances = pdist2(KidArrSFM.ActualPos, BalArrSFM.ActualPos);
            [~, currentGoal] = min(distances, [], 2);
            
            for i = 1:KidArrSFM.N   
                % if currently targeted balloon has been visited before
                row_of_ID = find(ismember(KidArrSFM.ID,KidArrSFM.ID(currentGoal(i))));
                if ismember(KidArrSFM.ID(row_of_ID),KidArrSFM.BalVisited(KidArrSFM.ID(i),:))
                    % [~,c_idx] = find(ismember(indices(i), ...
                    %                       KidArrSFM.BalVisited(KidArrSFM.ID(i),:)));
                    % set all visited balloons in distance array to inf, so the
                    % next closest will be chosen as next goal
                    nonzero = find(KidArrSFM.BalVisited(KidArrSFM.ID(i),:)); % find all non zero elements
                    set2inf = find(ismember(KidArrSFM.ID, KidArrSFM.BalVisited(KidArrSFM.ID(i),nonzero)));
                    distances(i, set2inf) = inf;
                    
                end
            end
            [~, currentGoal] = min(distances, [], 2);   % determine next closest balloon
        
            KidArrSFM.Destinations = BalArrSFM.ActualPos(currentGoal, :);
            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
    
    
        elseif params.Subcase == 2
            % kids send messages to the others (as a first step send to all others)
            % => X reaches Y, lets all others know about Y => Y goes to Y, all
            % others ignore/avoid Y
            %
            % First step is the same: see above
            %
            % Since we've already set destinations in the main we don't want to
            % overwrite them now, we just want to add the closest balloon to
            % those kids that don't have an updated destination
            %
            % distances = pdist2(KidArrSFM.ActualPos, BalArrSFM.ActualPos); % InitPos
            % [~, currentGoal] = min(distances, [], 2);   % currentGoal contains ID!
            % 
            % for i = 1:KidArrSFM.N   % = only those that are still on the run
            %     row_of_ID = find(ismember(KidArrSFM.ID,currentGoal(i))); % wos is der sinn von dera zeile gwesn?
            %                 % i moan dass i do davo ausganga bin dass
            % 
            %     % if currently targeted balloon has been visited before
            %     if ismember(KidArrSFM.ID(row_of_ID),KidArrSFM.BalVisited(KidArrSFM.ID(i),:))
                    % set all visited balloons in distance array to inf, so the
                    % next closest will be chosen as next goal
            %         nonzero = find(KidArrSFM.BalVisited(KidArrSFM.ID(i),:)); % find all non zero elements
            %         set2inf = find(ismember(KidArrSFM.ID, KidArrSFM.BalVisited(KidArrSFM.ID(i),nonzero)));
            %         distances(i, set2inf) = inf;
            %         [~, currentGoal(i)] = min(distances(i,:), [], 2);   % determine next closest balloon only for this kid
            % % problem: sometimes first if loop is not entered and second if
            % % loop can't be reached
            % % => base balvisited on the actual pos of balloons and not on
            % 
            %         if (~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))) || ...
            %            (~ismember(KidArrSFM.Destinations(i,:),BalArrSFM.ActualPos,'rows'))                  
            %             % Next closest balloon is set as the new destination,
            %             % only if the kid has not been communicated the actual
            %             % position of their balloon, or if their current target
            %             % "doesn't exist anymore"/has been found by its
            %             % respective kid
            %             KidArrSFM.Destinations(i,:) = BalArrSFM.ActualPos(currentGoal(i), :);
            %         end
            %     end
            % end
    
            % second try => seems to work
            distances = pdist2(KidArrSFM.ActualPos, BalArrSFM.InitPos); % InitPos
            [~, currentGoal] = min(distances, [], 2);   % currentGoal contains Bal-ID!
    
            for i = 1:KidArrSFM.N   % = only those that are still on the run
                           
                % check if currently targeted balloon has been visited before
                if ismember(currentGoal(i),KidArrSFM.BalVisited(KidArrSFM.ID(i),:))
                    % set all visited balloons in distance array to infinity,
                    % so the next closest will be chosen as next goal
                    nonzero = find(KidArrSFM.BalVisited(KidArrSFM.ID(i),:)); % find all non zero elements
                    set2inf = KidArrSFM.BalVisited(KidArrSFM.ID(i),nonzero);
                    distances(i, set2inf) = inf;
                    [~, currentGoal(i)] = min(distances(i,:), [], 2);   % determine next closest balloon only for this kid
            
                    if (~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))) || ...
                       (~ismember(KidArrSFM.Destinations(i,:),BalArrSFM.ActualPos,'rows'))                  
                        % Next closest balloon is set as the new destination,
                        % only if the kid has not been communicated the actual
                        % position of their balloon, or if their current target
                        % "doesn't exist anymore"/has been found by its
                        % respective kid
                        KidArrSFM.Destinations(i,:) = BalArrSFM.InitPos(currentGoal(i), :);
                    end
                end
            end
            
            % KidArrSFM.Destinations(find(ismember(KidArrSFM.ID,KidArrSFM.ID_arr)),:) = ...
    
    
            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
    
    
            % params.flagForce = 0; % can i do this just for a few balloons??
                                    % Why did i want to??
    
        end
        
    
    
    
    %% Case 3    
    elseif params.Case == 3 
        % sent message needs to be the current estimated position of the kid
        % that entered in the proximity of a balloon.
        
        params.WithEstimatedPos = 1;

        if params.Subcase == 1
            if params.NumSFMExec == 1 %s==1
                % set first destination randomly around the kid 
                for i = 1:KidArrSFM.N            
                    angle = 2*pi*rand;  % Generate a random angle   
                    % radius=1 at first. Watch out for destinations outside of room!
                    new_x = KidArrSFM.ActualPos(i, 1) + cos(angle);
                    new_y = KidArrSFM.ActualPos(i, 2) + sin(angle);            
                    KidArrSFM.Destinations(i,:) = [new_x, new_y];
                end
            else
                for i = 1:KidArrSFM.N
                    if ~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                        % movement "randomly straight", "exploration mode"
                        direc = normalize(KidArrSFM.ActualVel(i,:), 'norm', 2);
                        ang2hor = atan2(direc(:, 2), direc(:, 1));
                        angles = linspace(-0.25*pi, 0.25*pi, 7);       
                        rot_mat = [cos(ang2hor), -sin(ang2hor); ...
                                   sin(ang2hor),  cos(ang2hor)];
                        r_ang = randi(7);   % randomly choose one of the 7 new directions
                        xb = 2*cos(angles(r_ang));
                        yb = 2*sin(angles(r_ang));
            
                        [xy] = rot_mat*[xb;yb]; % rotate into base frame
            
                        KidArrSFM.Destinations(i,:) = ...
                            KidArrSFM.ActualPos(i,:) + [xy(1), xy(2)];
    
                        % check if new point lays within room's borders
                        [is_inside,exit] = isInside(KidArrSFM.Destinations(i,1), ...
                            KidArrSFM.Destinations(i,2), Room);
                        if ~is_inside
                            % search for closest of the 7 options to the inside
                            if exit == "top-right"
                                KidArrSFM.Destinations(i,2) = 0.9*Room.Height;
                                KidArrSFM.Destinations(i,1) = 0.9*Room.Width;
                            elseif exit == "bottom-right"
                                KidArrSFM.Destinations(i,2) = 0.1*Room.Height;
                                KidArrSFM.Destinations(i,1) = 0.9*Room.Width;
                            elseif exit == "bottom-left"
                                KidArrSFM.Destinations(i,2) = 0.1*Room.Height;
                                KidArrSFM.Destinations(i,1) = 0.1*Room.Width;
                            elseif exit == "top-left"
                                KidArrSFM.Destinations(i,2) = 0.9*Room.Height;
                                KidArrSFM.Destinations(i,1) = 0.1*Room.Width;
                            elseif exit == "top"
                                KidArrSFM.Destinations(i,2) = 0.9*Room.Height;
                            elseif exit == "right"
                                KidArrSFM.Destinations(i,1) = 0.9*Room.Width;
                            elseif exit == "bottom"
                                KidArrSFM.Destinations(i,2) = 0.1*Room.Height;
                            elseif exit == "left"
                                KidArrSFM.Destinations(i,1) = 0.1*Room.Width;
                            end
                        end
        
        
                    elseif KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                        % do nothing, Destination has been set in main
                        % KidArrSFM.Destinations(i,:) = ...
                        %     KidArrSFM.ActualPos(i,:) + [xy(1), xy(2)];
        
                        % later: take average of all sent positions if more arrive
        
                    end
        
                end
            end    
    
            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
        
        elseif params.Subcase == 2
        % Random flock movement
        % choose the next random position according to the same procedure
        % from case 3.1
            flockPos = mean(KidArrSFM.ActualPos,1);
            if params.NumSFMExec == 1 %s==1
                % set first destination to the CoM of all kids to form flock.
                % Remember that rows are height of room    
                for i = 1:KidArrSFM.N              
                    KidArrSFM.Destinations(i,:) = flockPos;
                end
            else   
                prevDes = mean(KidArrSFM.Destinations(find(ismember( ...
                    KidArrSFM.ID,find(KidArrSFM.FlagPosReceived==0))),:),1);

                % keep targeting the previous position if not yet explored by at least one kid
                if ~any(find(all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[1 1],2)))
                    newDest = prevDes;
                else
                    % set destination randomly around the kids                                 
                    % angle = 2*pi*rand;  % Generate a random angle   
                    % radius = Room.Width/3;  % radius
                    % new_x = flockPos(1) + radius*cos(angle); 
                    % new_y = flockPos(2) + radius*sin(angle);            
                    % newDest = [new_x, new_y];
                    % 

                    % is_inside = 0;
                    % while is_inside == 0
                    % 
                        % direc = normalize(prevDes-flockPos, 'norm', 2);
                        % ang2hor = atan2(direc(:, 2), direc(:, 1));
                        % angles = linspace(-0.25*pi, 0.25*pi, 7);       
                        % rot_mat = [cos(ang2hor), -sin(ang2hor); ...
                        %            sin(ang2hor),  cos(ang2hor)];
                        % r_ang = randi(7);   % randomly choose one of the 7 new directions
                        % radius = Room.Width/6;  % radius/look-ahead
                        % xb = radius*cos(angles(r_ang));  
                        % yb = radius*sin(angles(r_ang));
                        % 
                        % [xy] = rot_mat*[xb;yb]; % rotate into base frame
                        % newDest = flockPos + [xy(1), xy(2)];

                        % [is_inside,~] = isInside(newDest(1), newDest(2), Room);
                    % end

                    direc = normalize(prevDes-flockPos, 'norm', 2);
                    ang2hor = atan2(direc(:, 2), direc(:, 1));
                    angles = linspace(-0.25*pi, 0.25*pi, 7);       
                    rot_mat = [cos(ang2hor), -sin(ang2hor); ...
                               sin(ang2hor),  cos(ang2hor)];
                    radius = Room.Width/6;  % radius/look-ahead
                    choosePos = 1;
                    while choosePos                        
                        xb = radius*cos(angles);  
                        yb = radius*sin(angles);    
                        [xy] = rot_mat*[xb;yb]; % rotate into base frame
                        newDest = flockPos + [xy(1,:)', xy(2,:)'];
                        for i = 1:7
                            [is_inside,~] = isInside(newDest(i,1), newDest(i,2), Room);
                            if ~is_inside
                                if i<=3
                                    % rotate to right by a little
                                    angles = angles + 0.1*pi;
                                else
                                    % rotate to left
                                    angles = angles - 0.1*pi;
                                end
                                break
                            end
                            choosePos = 0;
                        end
                    end
                    
                    newDest = newDest(randi(7),:); % randomly choose one of the 7 new directions

    
                    % check if new point lays within room's borders
                    % [is_inside,exit] = isInside(newDest(1), newDest(2), Room);
                    % if ~is_inside
                    %     % search for closest of the 7 options to the inside
                    %     if exit == "top-right"
                    %         newDest(2) = 0.9*Room.Height;
                    %         newDest(1) = 0.9*Room.Width;
                    %     elseif exit == "bottom-right"
                    %         newDest(2) = 0.1*Room.Height;
                    %         newDest(1) = 0.9*Room.Width;
                    %     elseif exit == "bottom-left"
                    %         newDest(2) = 0.1*Room.Height;
                    %         newDest(1) = 0.1*Room.Width;
                    %     elseif exit == "top-left"
                    %         newDest(2) = 0.9*Room.Height;
                    %         newDest(1) = 0.1*Room.Width;
                    %     elseif exit == "top"
                    %         newDest(2) = 0.9*Room.Height;
                    %     elseif exit == "right"
                    %         newDest(1) = 0.9*Room.Width;
                    %     elseif exit == "bottom"
                    %         newDest(2) = 0.1*Room.Height;
                    %     elseif exit == "left"
                    %         newDest(1) = 0.1*Room.Width;
                    %     end
                    % end

                    newDest
                end

                

                for i = 1:KidArrSFM.N
                    if ~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                        % assign the next flock target to all kids that are
                        % still in search of their balloon
                        KidArrSFM.Destinations(i,:) = newDest;
    
                    elseif KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                        % do nothing, correct final destination has been
                        % set in main
    
                    end
                end

            end

            KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
        end

    %% CASE 4

    elseif params.Case == 4 && params.Subcase == 1

        params.WithEstimatedPos = 1;
        % Robots exploring the unvisited areas in a flock. Repulsive
        % forces in SFM should deal with the flocking and avoid
        % collisions
        if params.NumSFMExec == 1 %s==1
            % set first destination to the CoM of all kids to form flock.
            % Remember that rows are height of room
            flockPos = mean(KidArrSFM.ActualPos);    
            for i = 1:KidArrSFM.N              
                KidArrSFM.Destinations(i,:) = flockPos;
                % params.lastChosenIdx = 1;   % for later, see OccupancyMap
            end
        else                
            % first find new flock target
            [nextPos, params] = TestOccupancyMap(KidArrSFM, Room, params);

            for i = 1:KidArrSFM.N
                if ~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                    % assign the next flock target to every kid that's
                    % still in search of its balloon
                    KidArrSFM.Destinations(i,:) = nextPos;

                elseif KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))
                    % do nothing, correct final destination has been
                    % set in main

                end
            end
        end
    

        KidArrSFM.InitCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
        
        
    else
        warning("Enter a valid case!")
    end
end




%% All the stuff that happens after the last / before the next call to SFM

if callerFunction == "SKWTB_main"

    %% Estimate positions of kids with sensors
    if params.WithEstimatedPos
        % The updated positions that we obtain from the SFM are the new
        % actual positions after we've shifted the initial
        % in these cases we need to estimate t
        KidArrSFM.EstimatedPos = EstimatePosition(KidArrSFM, Sensor, Room, false);
    end

    %% Save previous positions
    % Use for the metric plots later
    % oldPos = KidArrMetric.ActualPos; % KidArrSFM.OldPos
    % oldVel = sqrt(sum(KidArrMetric.ActualVel.^2, 2)); % KidArrSFM.OldVel
    % % Update the final KidArray with new values of SFM
    % for i = 1:KidArrSFM.N       
    %     % Update Position and actual velocity
    %     KidArrMetric.ActualPos(KidArrSFM.ID(i),:) = KidArrSFM.ActualPos(i,:);
    %     KidArrMetric.ActualVel(KidArrSFM.ID(i),:) = KidArrSFM.ActualVel(i,:);  
    % end
    % % Estimate total path length and time it took to reach goal
    % deltaDistance = diag(pdist2(KidArrMetric.ActualPos, oldPos));
    % KidArrMetric.PathLength = KidArrMetric.PathLength + deltaDistance;
    % newVel = sqrt(sum(KidArrMetric.ActualVel.^2, 2));
    % avgVel = (oldVel + newVel)/2;
    % KidArrMetric.TravelTime = KidArrMetric.TravelTime + deltaDistance./avgVel;

    % Use for the metric plots later
    oldPos = KidArrSFM.OldPos;
    oldVel = sqrt(sum(KidArrSFM.OldVel.^2, 2)); % 
    % Update the old positions with the new actual values from SFM
    for i = 1:KidArrSFM.N       
        % Update Position and actual velocity
        KidArrSFM.OldPos(KidArrSFM.ID(i),:) = KidArrSFM.ActualPos(i,:);
        KidArrSFM.OldVel(KidArrSFM.ID(i),:) = KidArrSFM.ActualVel(i,:);  
    end
    % Estimate total path length and time it took for last simulation step
    deltaDistance = diag(pdist2(KidArrSFM.OldPos, oldPos)); % keep in mind that the first is already updated!
    KidArrSFM.PathLength = KidArrSFM.PathLength + deltaDistance;
    newVel = sqrt(sum(KidArrSFM.OldVel.^2, 2));
    avgVel = (oldVel + newVel)/2;
    KidArrSFM.TravelTime = KidArrSFM.TravelTime + deltaDistance./avgVel;
  
    %% Check if any kid has reached a balloon
    if (params.Case == 1) || (params.Case == 2)
        touch = KidArrSFM.Radius + BalArrSFM.Edge/2;
        arrived = all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[touch touch],2);
    elseif (params.Case == 3) || (params.Case == 4)
        touch = KidArrSFM.Radius + BalArrSFM.Edge;    % choose a bit bigger here to try out
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
            if params.print_flag
                fprintf("Kids that have reached their balloon: ");
                params.print_flag = 0;
            end
            fprintf("%d ", ID_KidsArrived');


            % Reveal color and ID of the balloon   => here not because they
            % know where to go
            % for i = 1:length(ID_KidsArrived)
            %     set(BalArrSFM.plotBalID(ID_KidsArrived(i)), 'Visible', 'on');
            %     set(BalArrSFM.squarefig(ID_KidsArrived(i)), 'FaceColor', ...
            %         KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_KidsArrived(i)))),:));   
            % end  
            

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

            % Update the numbers
            KidArrSFM.N = KidArrSFM.N - length(ID_KidsArrived);
            BalArrSFM.N = BalArrSFM.N - length(ID_KidsArrived);            
            
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
                    set(BalArrSFM.plotBalID(ID_Bal2Check), 'Visible', 'on');
                    set(BalArrSFM.squarefig(ID_Bal2Check), 'FaceColor', ...
                        KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_Bal2Check))),:));                    
                end


                if (ID_KidsArrived(h) == ID_Bal2Check)

                    if params.print_flag
                        fprintf("Kids that have reached their balloon: ");
                        params.print_flag = 0;
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
            

                else % (ID_KidsArrived(h) ~= ID_Bal2Check), no match
                    
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

        %% CASE 3 & 4
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
       
        if (params.Case == 3) || (params.Case == 4)
            % 
            for h = 1:length(ID_KidsArrived)

                % get row of arrived kid
                rKidID_arr = find(ismember(KidArrSFM.ID, ID_KidsArrived(h), 'rows'));
                [~,ID_Bal2Check] = min(sum(abs(KidArrSFM.ActualPos(rKidID_arr,:) - ...
                                BalArrSFM.InitPos),2));
                
                % Reveal color and ID of the balloon  
                if ismember(ID_Bal2Check, KidArrSFM.ID)
                    set(BalArrSFM.plotBalID(ID_Bal2Check), 'Visible', 'on');
                    set(BalArrSFM.squarefig(ID_Bal2Check), 'FaceColor', ...
                        KidArrSFM.Color((find(ismember(KidArrSFM.ID,ID_Bal2Check))),:));                    
                end


                if (ID_KidsArrived(h) == ID_Bal2Check)

                    if params.print_flag
                        %fprintf("Kids that have reached their balloon: ");
                        fprintf("I'm kid number ")
                        % params.print_flag = 0;
                    end
                    fprintf("%d and I've reached my balloon. Yippee!", ID_KidsArrived(h));
                    fprintf("\n");

                    % plot the arrived kid again
                    if ~params.plotTrajEst
                        i = rKidID_arr;                        
                        x_min = KidArrSFM.ActualPos(i,1) - KidArrSFM.Radius;
                        y_min = KidArrSFM.ActualPos(i,2) - KidArrSFM.Radius;
                        rad = KidArrSFM.Radius;
                        KidArrSFM.circlefig(ID_KidsArrived(h)) = rectangle('Position',[x_min,y_min,2*rad,2*rad],...
                            'Curvature',[1 1], 'FaceColor',KidArrSFM.Color(i,:));   
                        KidArrSFM.text(ID_KidsArrived(h)) = text(KidArrSFM.ActualPos(i,1), KidArrSFM.ActualPos(i,2), num2str(ID_KidsArrived(h)), ...
                            'HorizontalAlignment', 'center', 'Color','k', 'FontSize', rad*15);            
                    end

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

       

    end


end

end 
