function [KidArrSFM, BalArrSFM, params] = ...
                        SFM2(KidArrSFM, BalArrSFM, Sensor, Room, params)
% Summary: Social Force Model and different cases of analysis
% Description: implementation of the Social Force Model according to the paper 
% "Parameter Calibration of a Social Force Model for the Crowd-Induced Vibrations
% of Footbridges" by Elisa Bassoli and Loris Vincenzi
    % NOTE: alpha from the paper becomes k
    %       beta from the paper becomes j

% Return values in KidArrSFM that get updated are:
    % - Positions
    % - ActualVel


%% Counter for number of function calls during one run
% Necessary to execute some parts of the code only at first call
% persistent s 
% if isempty(s)
%     s = 0;
% end
% s = s + 1;

params.NumSFMExec = params.NumSFMExec + 1;

%% Case distinction
% set initial conditions for the separate cases
% KidArrSFM.InitCond is the mainly important return value
[KidArrSFM, BalArrSFM, params] = ...
                    CaseDistinction(KidArrSFM, BalArrSFM, Sensor, Room, params);


%% Optimization step

% Before overwriting the ActualPos save it here in intermediate variable
PrevActualPos = KidArrSFM.ActualPos;

% Set simulation time
tSpan = [0, params.t];

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);
%tic
[t, y] = ode113(@(t,y) socialForceModel(t, y, KidArrSFM, BalArrSFM, Room, params), ...
                    tSpan, KidArrSFM.InitCond, options);

% Extract results
KidVelX = reshape(y(:,               1:   KidArrSFM.N)', KidArrSFM.N, []);
KidVelY = reshape(y(:,   KidArrSFM.N+1: 2*KidArrSFM.N)', KidArrSFM.N, []);
KidPosX = reshape(y(:, 2*KidArrSFM.N+1: 3*KidArrSFM.N)', KidArrSFM.N, []);
KidPosY = reshape(y(:, 3*KidArrSFM.N+1:           end)', KidArrSFM.N, []);

% Save the last values (end) into the arrays as their new current values
KidArrSFM.ActualPos = [KidPosX(:,end),  KidPosY(:,end)];
KidArrSFM.ActualVel = [KidVelX(:,end),  KidVelY(:,end)];



%% Shift the estimated path into the known previous position
% We start from known initial starting positions. Based on their estimate,
% we obtain the forces and the paths. These then need to be shifted into
% the known positions to obtain the actual real next positions

if params.WithEstimatedPos
    % The last actual known position is saved in the first two columns of
    % ActualPos => compute translation in x and y in comparison to its estimate
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
figure(1)

if params.plotTrajEst
    % means the trajectory, made of many intermediate points, is plotted.
    % Those points are of the same color but smaller than the starting
    % position. The estimates are plotted aswell

    % determine # of gobjects needed for plot
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
    
    % KidArrSFM.circlefig = zeros(1,KidArrSFM.N);
    for i = 1:KidArrSFM.N    
        % intermediate points smaller and without number
        rad = KidArrSFM.Radius/3;
        x_min = KidArrSFM.ActualPos(i,1) - rad;
        y_min = KidArrSFM.ActualPos(i,2) - rad;
        KidArrSFM.circlefig(i) = rectangle('Position',[x_min,y_min,2*rad,2*rad],...
        'Curvature',[1 1], 'FaceColor',KidArrSFM.Color(i,:));   
    
        if params.WithEstimatedPos
            % plot also the estimated position for better understanding (debugging)
            x_e = KidArrSFM.EstimatedPos(i,1) - rad;
            y_e = KidArrSFM.EstimatedPos(i,2) - rad;
            rectangle('Position',[x_e,y_e,2*rad,2*rad],...
            'Curvature',[1 1], 'EdgeColor', KidArrSFM.Color(i,:));   
        end
    end
else    
    % ^= plot actualPos. To see the flocking behavior better, only the
    % current actual positions are plotted    

    % Preparation of returned arrays
    counter = 0;    % to not take every frame into the video
    l = length(t);
    KidPosX = KidPosX(:, 1:floor(1 + l/100):end); % shorten the point array
    KidPosY = KidPosY(:, 1:floor(1 + l/100):end); % to size Nxl
    
    for i = 1:KidArrSFM.N
        delete(KidArrSFM.circlefig(KidArrSFM.ID(i)))
        delete(KidArrSFM.text(KidArrSFM.ID(i)))
    end

    for j = 1:length(KidPosX)
        for i = 1:KidArrSFM.N    
            % intermediate points smaller and without number
            rad = KidArrSFM.Radius;
            x_min = KidPosX(i,j) - rad;
            y_min = KidPosY(i,j) - rad;
            KidArrSFM.circlefig(KidArrSFM.ID(i)) = rectangle('Position',[x_min,y_min,2*rad,2*rad],...
                'Curvature',[1 1], 'FaceColor',KidArrSFM.Color(i,:));   
            KidArrSFM.text(KidArrSFM.ID(i)) = text(KidPosX(i,j), KidPosY(i,j), num2str(KidArrSFM.ID(i)), ...
                'HorizontalAlignment', 'center', 'Color','k', 'FontSize', rad*15);            
            
        end
        % Wait and delete previous figure elements
        pause(0.0001);

        % Build up the video
        if params.VideoFlag && counter == 10
            frame = getframe(gcf);   % Capture the current frame 
            params.frames{end+1} = frame;  
            counter = 0;
        end
        counter = counter + 1;

        
        figure(1)
        if j ~= length(KidPosX)
            for i = 1:KidArrSFM.N
                delete(KidArrSFM.circlefig(KidArrSFM.ID(i)))
                delete(KidArrSFM.text(KidArrSFM.ID(i)))
            end
        end        
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
    
    A = 0.5;%0.5; %           % repulsive interaction strength, see table 1
    B = 0.1;%0.1; %           % repulsive interaction range, see table 1
    r_kj = 2+2*KidArrSFM.Radius;     % sum of their radii
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
    
    
    %% Equation 6a), Attraction force f_ka
        
    % "These attractive forces can be modeled accordingly to the
    % repulsive forces among pedestrians of Eq. 6 with longer
    % interaction range and negative interaction strength."
    f_ka = zeros(KidArrSFM.N,2);
    
    A = -0.5;%-0.5; %        % attractive interaction strength, sign inverted
    B = 0.5;%  0.5; %       % attractive interaction range, >0.1 => modifiy and see what works
    r_kj = 2+2*KidArrSFM.Radius;     % sum of their radii
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
        

    
    %% Equation 7, Repulsion from borders f_kb
    
    % intention to keep a certain distance from borders. The repulsive effect
    % of borders is similar to the repulsion among pedestrians except for the
    % anisotropic behavior.
    f_kb = zeros(KidArrSFM.N,2);  % force in x and y direction on kid k (Nx2)
    d_kb = zeros(2);        % initialization, 1 for x, 2 for y
    idx = zeros(1,2);
    A = 0.2;%  0.5; %              % repulsive interaction strength, see table 1
    B = 0.1;%   0.1; %             % repulsive interaction range, see table 1
    
    for k=1:KidArrSFM.N
        
            % simple room with 4 walls => make more general later
            % here it's not yet exactly like in the paper, because the
            % paper sums the influence of all walls, while this code
            % only takes the closest wall in every direction into account
            [d_kb(1,1),idx(1)] = min([abs(pos(k,1) - Room.Width), ...
                                    abs(pos(k,1) - 0)]);   % distance in x
            [d_kb(2,2),idx(2)] = min([abs(pos(k,2) - Room.Height), ...
                                    abs(pos(k,2) - 0)]);     % distance in y
    
        for b = 1:2
            n_kb = (-1)^idx(b) * normalize(d_kb(b,:).', 'norm', 2).'; % 
            f_kb(k,:) = f_kb(k,:) + A*exp((3*KidArrSFM.Radius - norm(d_kb(b,:)))/B) * n_kb;
           
        end
    end
        
    
    %% Own addition, Repulsion from balloons not targeted by a kid
    % prevents that kids k cross/run over a balloon x which is not its target.
    % Repulsive term prevents this 'physical constraint'
    % important to do so only with those that have been revealed yet
    f_kx = zeros(KidArrSFM.N,2);

    if params.flagForce        
        A = 0.15;            % repulsive interaction strength, similar to paper
        B = 0.1;            % repulsive interaction range, see table 1
        r_kx = (KidArrSFM.Radius + BalArrSFM.Edge);     % sum of their radii
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
    
    %% Define the system of ODEs (Equations 1 to 3)    
    % should we actually add the noise here or only in the estimate script??
    noise = (2*rand(size(KidArrSFM.ActualPos)) - 1)*1e-2;

    f_k = f_k0 + f_kj + f_kx + f_kb + f_ka + noise;

    
    % limit maximum value to prevent rocket launch (rebounce) 
    % from borders
    if any(abs((f_kb)) > 1.5)
        for k = 1:KidArrSFM.N
            if abs(f_kb(k,1)) > 1.5
                f_kb(k,1) = (f_kb(k,1))./abs(f_kb(k,1)) * 1.5;
            end
            if abs(f_kb(k,2)) > 1.5
                f_kb(k,2) = (f_kb(k,2))./abs(f_kb(k,2)) * 1.5;
            end
        end
        f_k = f_k0 + f_kj + f_ka + f_kx + f_kb + noise;
    end

    %from balloons
    if any(abs((f_kx)) > 1.5)
        for k = 1:KidArrSFM.N
            if abs(f_kx(k,1)) > 1.5
                f_kx(k,1) = (f_kx(k,1))./abs(f_kx(k,1)) * 1.5;
            end
            if abs(f_kx(k,2)) > 1.5
                f_kx(k,2) = (f_kx(k,2))./abs(f_kx(k,2)) * 1.5;
            end
        end
        f_k = f_k0 + f_kj + f_ka + f_kx + f_kb + noise;
    end

    % for repulsion and attraction    
    if any(abs((f_kj+f_ka)) > 1.5)
        f_rep_att = zeros(size(f_kj));
        for k = 1:KidArrSFM.N
            if abs((f_kj(k,1)+f_ka(k,1))) > 1.5
                f_rep_att(k,1) = ...
                    (f_kj(k,1)+f_ka(k,1))./abs(f_kj(k,1)+f_ka(k,1)) * 1.5;
            end
            if abs((f_kj(k,2)+f_ka(k,2))) > 1.5
                f_rep_att(k,2) = ...
                    (f_kj(k,2)+f_ka(k,2))./abs(f_kj(k,2)+f_ka(k,2)) * 1.5;
            end
        end
        f_k = f_k0 + f_rep_att + f_kx + f_kb + noise;
    end

    

    % slow down kids if close to a balloon to prevent overshoot    
        % seems to work, but it also slows down the script a lot
    % if params.Case ~= 3
    %     for k=1:KidArrSFM.N
    %         if all(abs(KidArrSFM.ActualPos(k,:) - KidArrSFM.Destinations(k,:))<[1.5 1.5],2)
    %             f_k(k,1) = f_k(k,1)./abs(f_k(k,1)) * 0.85;            
    %             f_k(k,2) = f_k(k,2)./abs(f_k(k,2)) * 0.85;           
    %         end
    %     end
    % end
    
    
    %% Combine velocity and position derivatives into a single vector
    dydt = [f_k(:); vel(:)];



end