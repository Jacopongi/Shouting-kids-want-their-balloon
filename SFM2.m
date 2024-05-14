function [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params)
% Implementation of the Social Force Model according to the paper 
% "Parameter Calibration of a Social Force Model for the Crowd-Induced 
% Vibrations of Footbridges" by Elisa Bassoli and Loris Vincenzi
    % alpha from the paper becomes k
    % beta from the paper becomes j
% Return values in KidArrSFM that get updated are:
    % Positions
    % ActualVel


%% Counter for number of function calls during one run
persistent s 
if isempty(s)
    s = 0;
end
s = s + 1;

%% Case distinction

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
%}

% !!! => use flags to de-/activate forces for different cases (in params) ???

if params.Case == 1
    % Every kid knows which balloon is theirs and runs towards it
    KidArrSFM.Destinations = BalArrSFM.ActualPos;
    if params.Subcase == 1
        % Combine initial positions and velocities into a single vector
        % Use absolute positions as reference
        initCond = [KidArrSFM.ActualVel(:); KidArrSFM.ActualPos(:)];
    elseif params.Subcase == 2
        % Use estimated positions     
        initCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];
    end

elseif params.Case == 2 
    if (params.Subcase == 1) || (params.Subcase == 2 && s == 1)

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
        initCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];


    elseif params.Subcase == 2
        % kids send messages to the others (as a first step send to all others)
        % => X reaches Y, lets all others know about Y => Y goes to Y, all
        % others ignore/avoid Y
        
        % First step is the same: see above
        
        % Since we've already set destinations in the main we don't want to
        % overwrite them now, we just want to add the closest balloon to
        % those kids that don't have an updated destination

        distances = pdist2(KidArrSFM.ActualPos, BalArrSFM.ActualPos);
        [~, currentGoal] = min(distances, [], 2);   % currentGoal contains ID!

        for i = 1:KidArrSFM.N   % = only those that are still on the run
            row_of_ID = find(ismember(KidArrSFM.ID,currentGoal(i)));
            
            % if currently targeted balloon has been visited before
            if ismember(KidArrSFM.ID(row_of_ID),KidArrSFM.BalVisited(KidArrSFM.ID(i),:))
                % set all visited balloons in distance array to inf, so the
                % next closest will be chosen as next goal
                nonzero = find(KidArrSFM.BalVisited(KidArrSFM.ID(i),:)); % find all non zero elements
                set2inf = find(ismember(KidArrSFM.ID, KidArrSFM.BalVisited(KidArrSFM.ID(i),nonzero)));
                distances(i, set2inf) = inf;
                [~, currentGoal(i)] = min(distances(i,:), [], 2);   % determine next closest balloon only for this kid
        % problem: sometimes first if loop is not entered and second if
        % loop can't be reached
        % => base balvisited on the actual pos of balloons and not on

                if (~KidArrSFM.FlagPosReceived(KidArrSFM.ID(i))) || ...
                   (~ismember(KidArrSFM.Destinations(i,:),BalArrSFM.ActualPos,'rows'))                  
                    % Next closest balloon is set as the new destination,
                    % only if the kid has not been communicated the actual
                    % position of their balloon, or if their current target
                    % "doesn't exist anymore"/has been found by its
                    % respective kid
                    KidArrSFM.Destinations(i,:) = BalArrSFM.ActualPos(currentGoal(i), :);
                end
            end
        end
        
        % KidArrSFM.Destinations(find(ismember(KidArrSFM.ID,KidArrSFM.ID_arr)),:) = ...


        initCond = [KidArrSFM.ActualVel(:); KidArrSFM.EstimatedPos(:)];


        % params.flagForce = 0; % can i do this just for a few balloons??

    end
    
elseif params.Case == 3 
    % !! find a way to neglect f_k0 in final step when we dont know the
    % balloon position
else
    warning("Enter a valid case !")
end


%% Optimization step

% Before overwriting the ActualPos save it here in intermediate variable
PrevActualPos = KidArrSFM.ActualPos;

% Set simulation time
tSpan = [0, params.t];    % for example

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);
%tic
[t, y] = ode113(@(t,y) socialForceModel(t, y, KidArrSFM, BalArrSFM, Room, params), ...
                    tSpan, initCond, options);

% Extract results
KidVelX = reshape(y(:,               1:   KidArrSFM.N)', KidArrSFM.N, []);
KidVelY = reshape(y(:,   KidArrSFM.N+1: 2*KidArrSFM.N)', KidArrSFM.N, []);
KidPosX = reshape(y(:, 2*KidArrSFM.N+1: 3*KidArrSFM.N)', KidArrSFM.N, []);
KidPosY = reshape(y(:, 3*KidArrSFM.N+1:           end)', KidArrSFM.N, []);

% Save the last values (end) into the arrays as their new current values
KidArrSFM.ActualPos = [KidPosX(:,end),  KidPosY(:,end)];
KidArrSFM.ActualVel = [KidVelX(:,end),  KidVelY(:,end)];



%% Shift the estimated path into the known previous position
% We start from known initial starting position. Based on its estimate, we
% obtain the forces and the path. This then needs to be shifted into the
% known position to obtain our next actual position

if (params.Case == 1 && params.Subcase == 2) || (params.Case == 2) 
    % The last actual known position is saved in the first two columns of
    % ActualPos => compute translation in x and y between this and its estimate
    XY_Distance = PrevActualPos - KidArrSFM.EstimatedPos;
    
    % These deviations now need to be applied to all points that create the
    % paths of all the kids.
    % The plot is done with animatedline which takes KidPosX and KidPosY
    % separately => simple shift
    KidPosX = KidPosX + XY_Distance(:,1);
    KidPosY = KidPosY + XY_Distance(:,2);
    
    % Save new ActualPos to have the correct starting point for the next step
    KidArrSFM.ActualPos = [KidPosX(:,end),  KidPosY(:,end)];
end

%% Plot the results
figure(5), axis equal
axis([1, Room.Width, 1, Room.Height]);
xlabel('X Position');
ylabel('Y Position');
title('Shouting kids want their balloon');
subtitle(append('Case: ',num2str(params.Case),'.',num2str(params.Subcase)));

% Plot balloon squares only at first function call
if s == 1   
    squarefig = zeros(1,BalArrSFM.N);
    for i = 1:BalArrSFM.N
        x_min_b = BalArrSFM.ActualPos(i,1) - 0.5*BalArrSFM.Edge; 
        y_min_b = BalArrSFM.ActualPos(i,2) - 0.5*BalArrSFM.Edge;
        x_max_b = BalArrSFM.Edge;
        y_max_b = BalArrSFM.Edge;
        squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
            'FaceColor', KidArrSFM.Color(i,:));       
        text(BalArrSFM.ActualPos(i,1), BalArrSFM.ActualPos(i,2), num2str(BalArrSFM.ID(i)), ...
            'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalArrSFM.Edge*10);
    end

    for i = 1:KidArrSFM.N        
        % starting position bigger and with number. Plot only once! BUT:
        % s=1 is already the first step (actualpos~=initpos) => plot both
        % in this instance. Not so pretty but vabbè
        rad = KidArrSFM.Radius;
        x_min = KidArrSFM.InitPos(i,1) - rad;
        y_min = KidArrSFM.InitPos(i,2) - rad;
        KidArrSFM.circlefig(i) = rectangle('Position',[x_min,y_min,2*rad,2*rad],...
            'Curvature',[1 1], 'FaceColor',KidArrSFM.Color(i,:));
        text(KidArrSFM.InitPos(i,1), KidArrSFM.InitPos(i,2), num2str(KidArrSFM.ID(i)), ...
            'HorizontalAlignment', 'center', 'Color','k', 'FontSize', rad*10);
    end
end


% determine # of gobjects needed for plot
% if length(KidArrSFM.N) < 13     % otherwise too computationally demanding. If loop added after
                                % the inside, which should theoretically work for more, was written.
    m = floor(KidArrSFM.N/12);  % maximum 12 entries per set => m = # full sets (stack of 12)
    n = mod(KidArrSFM.N, 12);   % # > k*12 (number of kids that exceed a multiple of 12)
        % keep in mind to dynamically adjust the array size bc kids get
        % eliminated from the optimization once they reached their balloon

    for i = 1:m   
        name = strcat('set', num2str(i));     
        AL.(name) = gobjects(12,1);
        for j = 1:12 
            AL.(name)(j) = animatedline('Color', KidArrSFM.Color(j,:));
        end
    end
    if n~=0
        name = strcat('set', num2str(m+1)); 
        AL.(name) = gobjects(n,1);
        for j = 1:n 
            AL.(name)(j) = animatedline('Color', KidArrSFM.Color(12*m+j,:));
        end       
    end
    
    % Actually plotting the animated lines
    l = length(t);
    KidPosX = KidPosX(:, 1:floor(1 + l/100):end); % shorten the point array
    KidPosY = KidPosY(:, 1:floor(1 + l/100):end);

    for i = 1:length(KidPosX)
        for j = 1:m     
            name = strcat('set', num2str(j));
            for k = 1:12 
                addpoints(AL.(name)(k), KidPosX(k,i), KidPosY(k,i));
            end                                
        end

        if n~=0
            name = strcat('set', num2str(m+1));
            for k = 1:n 
                addpoints(AL.(name)(k), KidPosX(12*m+k,i), KidPosY(12*m+k,i));
            end
        end
        drawnow %limitrate;  % for faster animation

    end
% end
%}

% KidArrSFM.circlefig = zeros(1,KidArrSFM.N);
for i = 1:KidArrSFM.N    
    % intermediate points smaller and without number
    rad = KidArrSFM.Radius/3;
    x_min = KidArrSFM.ActualPos(i,1) - rad;
    y_min = KidArrSFM.ActualPos(i,2) - rad;
    KidArrSFM.circlefig(i) = rectangle('Position',[x_min,y_min,2*rad,2*rad],...
    'Curvature',[1 1], 'FaceColor',KidArrSFM.Color(i,:));   

    if (params.Case == 1 && params.Subcase == 2) || (params.Case == 2)
        % plot also the estimated position for better understanding (debugging)
        x_e = KidArrSFM.EstimatedPos(i,1) - rad;
        y_e = KidArrSFM.EstimatedPos(i,2) - rad;
        rectangle('Position',[x_e,y_e,2*rad,2*rad],...
        'Curvature',[1 1], 'FaceColor', "#808080", 'LineStyle', ":");   
    end
end





end



%%
function dydt = socialForceModel(t, y, KidArrSFM, BalArrSFM, Room, params)
    

    % Extract velocities and positions from the state vector
    vel = reshape(y(1:2*KidArrSFM.N), KidArrSFM.N, []);
    pos = reshape(y(2*KidArrSFM.N+1:end), KidArrSFM.N, []);

    %% Equation 4, Driving term f_k0
    % intention of each kid to walk with a desired speed v_k0 towards its
    % destination and the fact that deviations of the actual velocity v_k
    % from the desired velocity v0 are corrected within the relaxation time
    
    % Desired direction of motion (Equation 5)
    e_k = normalize((KidArrSFM.Destinations - pos).', 'norm', 2).';
        % instead of final destination this should be the 'next edge' on an
        % imaginary polygon => how to implement this? is it even necessary?
        % => come back to this for the different cases
    
    tau = 0.5;  % relaxation time (see table 1)
    epsilon = rand(size(vel))*1e-2;
    f_k0 = 1/tau*(KidArrSFM.DesiredVel.*e_k - vel + epsilon);  
   
    
    %% Equation 6, Repulsive force f_kj  
    % describes that the kid k tends to keep a situation-dependent
    % distance from the other kids j
    f_kj = zeros(KidArrSFM.N,2);
    
    A = 0.5;            % repulsive interaction strength, see table 1
    B = 0.1;            % repulsive interaction range, see table 1
    r_kj = 2*KidArrSFM.Radius;     % sum of their radii
    lambda_k = 0.5;     % anisotropic factor, see figure 1
    
    for k=1:KidArrSFM.N
        for j=1:KidArrSFM.N
            if j~=k
                d_kj = pos(k,:) - pos(j,:);    % distance between CoM
                n_kj = normalize(d_kj.', 'norm', 2).';
                f_kj(k,:) = f_kj(k,:) + A * exp((r_kj - norm(d_kj))/B) * n_kj * ...
                            (lambda_k + (1 - lambda_k)*0.5*(1 - n_kj*e_k(k,:)') + 1);
            end
        end
    end
        

    %% Own addition, Repulsion from balloons not targeted by a kid
    % prevents that kids k cross/run over a balloon x which is not its target.
    % Repulsive term prevents this 'physical constraint'
    f_kx = zeros(KidArrSFM.N,2);

    if params.flagForce        
        A = 0.35;            % repulsive interaction strength, similar to paper
        B = 0.1;            % repulsive interaction range, see table 1
        r_kx = KidArrSFM.Radius + BalArrSFM.Edge;     % sum of their radii
        lambda_k = 0.5;     % anisotropic factor, see figure 1
    
        % case 1: kid N belongs to balloon N => 
        for k=1:KidArrSFM.N
            for x=1:length(BalArrSFM.InitPos)     % => here we need the unshortened array
                if x~=KidArrSFM.ID(k)
                    d_kx = pos(k,:) - BalArrSFM.InitPos(x,:);    % distance between centers
                    n_kx = normalize(d_kx.', 'norm', 2).';
                    f_kx(k,:) = f_kx(k,:) + A * exp((r_kx - norm(d_kx))/B) * n_kx * ...
                                (lambda_k + (1 - lambda_k)*0.5*(1 - n_kx*e_k(k,:)') + 1);
                    
                end
            end
        end    
    end
    %% Equation 7, Repulsion from borders f_kb
    
    % intention to keep a certain distance from borders. The repulsive effect
    % of borders is similar to the repulsion among pedestrians except for the
    % anisotropic behavior.
    f_kb = zeros(KidArrSFM.N,2);  % force in x and y direction on kid k (Nx2)
    d_kb = zeros(2);        % initialization, 1 for x, 2 for y
    A = 0.5;                % repulsive interaction strength, see table 1
    B = 0.1;                % repulsive interaction range, see table 1
    
    for k=1:KidArrSFM.N
        
            % simple room with 4 walls => make more general later
            % here it's not yet exactly like in the paper, because the
            % paper sums the influence of all walls, while this code
            % only takes the closest wall in every direction into account
            d_kb(1,1) = min([abs(pos(k,1) - Room.Width), ...
                             abs(pos(k,1) - 0)]);   % distance in x
            d_kb(2,2) = min([abs(pos(k,2) - Room.Height), ...
                             abs(pos(k,2) - 0)]);     % distance in y
    
        for b = 1:2
            n_kb = normalize(d_kb(b,:).', 'norm', 2).';
            f_kb(k,:) = f_kb(k,:) + A*exp((KidArrSFM.Radius - norm(d_kb(b,:)))/B) * n_kb;
           
        end
    end
        
    
    %% Equation 6a), Attraction force f_ka
        
    % "These attractive forces can be modeled accordingly to the
    % repulsive forces among pedestrians of Eq. 6 with longer
    % interaction range and negative interaction strength."
    f_ka = zeros(KidArrSFM.N,2);
    
    A = -0.5;        % attractive interaction strength, sign inverted
    B = 0.5;         % repulsive interaction range, >0.1 => modifiy and see what works
    r_kj = 2*KidArrSFM.Radius;     % sum of their radii
    lambda_k = 0.5;     % anisotropic factor, see figure 1
    
    for k=1:KidArrSFM.N
        for j=1:KidArrSFM.N
            if j~=k
                d_kj = pos(k,:) - pos(j,:);    % distance between CoM
                n_kj = normalize(d_kj.', 'norm', 2).';
                f_ka(k,:) = f_ka(k,:) + A * exp((r_kj - norm(d_kj))/B) * n_kj * ...
                            (lambda_k + (1 - lambda_k)*0.5*(1 - n_kj*e_k(k,:)') + 1);
            end
        end
    end
    
    
    %% Define the system of ODEs (Equations 1 to 3)    
    % should we actually add the noise here or only in the estimate script??
    noise = (2*rand(size(KidArrSFM.ActualPos)) - 1)*1e-2;

    f_k = f_k0 + f_kj + f_kx + f_kb + f_ka + noise;

    % limit maximum value to prevent rocket launch
    
    if any(abs(f_k) > 1.5)
        for k=1:KidArrSFM.N
            if abs(f_k(k,1)) > 1.5
                f_k(k,1) = f_k(k,1)./abs(f_k(k,1)) * 1.5;
            end
            if abs(f_k(k,2)) > 1.5
                f_k(k,2) = f_k(k,2)./abs(f_k(k,2)) * 1.5;
            end
        end
    end
    
    %% Combine velocity and position derivatives into a single vector
    dydt = [f_k(:); vel(:)];



end