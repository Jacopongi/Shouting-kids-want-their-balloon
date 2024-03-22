function [KidArrSFM] = SFM2(KidArrSFM, BalArrSFM, Room, params)
% Implementation of the Social Force Model according to the paper 
% "Parameter Calibration of a Social Force Model for the Crowd-Induced 
% Vibrations of Footbridges" by Elisa Bassoli and Loris Vincenzi
    % alpha from the paper becomes k
    % beta from the paper becomes j
% Return values in KidArrSFM that get updated are:
    % Positions
    % ActualVel

% Counter for number of function calls during one run (for debugging)
persistent s 
if isempty(s)
    s = 0;
end
s = s + 1;



%% Case distinction

if params.Case == 1
    % every kid knows which balloon is theirs
    % !!! here we need to make sure that other balloons are recognized as
    % obstacles too and thus avoided.
    KidArrSFM.Destinations = BalArrSFM.Positions;
elseif params.Case == 2 
    % every kid runs to the nearest balloon
    distances = pdist2(KidArrSFM.Positions, BalArrSFM.Positions);
    [~, indices] = min(distances, [], 2);
    KidArrSFM.Destinations = BalArrSFM.Positions(indices, :);
else
    % !! find a way to neglect f_k0 in final step. when we dont know the
    % balloon position
end



%% Optimization step

% Combine initial positions and velocities into a single vector
initCond = [KidArrSFM.ActualVel(:); KidArrSFM.Positions(:)];

% Set simulation time
tSpan = [0, params.t];    % for example

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);
%tic
[t, y] = ode113(@(t,y) socialForceModel(t,y,KidArrSFM,BalArrSFM,Room), ...
                    tSpan, initCond, options);
%toc

% Extract results
KidVelX = reshape(y(:,               1:   KidArrSFM.N)', KidArrSFM.N, []);
KidVelY = reshape(y(:,   KidArrSFM.N+1: 2*KidArrSFM.N)', KidArrSFM.N, []);
KidPosX = reshape(y(:, 2*KidArrSFM.N+1: 3*KidArrSFM.N)', KidArrSFM.N, []);
KidPosY = reshape(y(:, 3*KidArrSFM.N+1:           end)', KidArrSFM.N, []);

KidArrSFM.Positions = [KidPosX(:,end), KidPosY(:,end)];
KidArrSFM.ActualVel = [KidVelX(:,end), KidVelY(:,end)];

%% Plot the results

%{
figure(3);
for i = 1:KidArray.N
    plot(KidPosX(i,:), KidPosY(i,:), 'DisplayName', ['Kid ', num2str(i)]);
    hold on;    
end

plot(KidArray.Positions(:,1), KidArray.Positions(:,2), 'ro')      % starting pos


axis equal
axis([0 Room.Width 0 Room.Height])
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Kids in the room','FontSize',14)

circlefig = zeros(1,KidArray.N);
KidArray.Color = rand(KidArray.N,3);
for i = 1:KidArray.N
    x_min = KidArray.Positions(i,1) - KidArray.Radius;
    y_min = KidArray.Positions(i,2) - KidArray.Radius;
    radius_cur = KidArray.Radius;
    circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
        'Curvature',[1 1], 'FaceColor', KidArray.Color(i,:));
end

squarefig = zeros(1,BalloonArray.N);

for i = 1:BalloonArray.N
    x_min_b = BalloonArray.Positions(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.Positions(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor', KidArray.Color(i,:));
end



% Review of the kids' movement to see their interaction
figure(4), clf, hold on
AL = gobjects(KidArray.N, 1);
for i = 1:KidArray.N
    AL(i) = animatedline('Color', KidArray.Color(i,:));
end
axis equal
axis([1, Room.Width, 1, Room.Height]);
% xlabel('X Position');
% ylabel('Y Position');
title('Reenactment of kids movement');

squarefig = zeros(1,BalloonArray.N);
for i = 1:BalloonArray.N
    x_min_b = BalloonArray.Positions(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.Positions(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor', KidArray.Color(i,:));       
end
for l = 1:length(t)
    for i = 1:KidArray.N
        % Add a point to the animated line for each kid
        addpoints(AL(i), KidPosX(i, l), KidPosY(i, l));
    end
    drawnow;        
end
%}

% Plot the results
%{
figure(3)
hold on
axis([1, Room.Width, 1, Room.Height]);
axis equal;
for i = 1:KidArray.N
    plot(KidPosX(i,:), KidPosY(i,:), 'DisplayName', ['Kid ', num2str(i)]);
end
plot(BalloonArray.Positions(:,1), BalloonArray.Positions(:,2), 'g*')  % balloons
plot(KidArray.Positions(:,1), KidArray.Positions(:,2), 'ro')      % starting pos
xlabel('X Position');
ylabel('Y Position');
%legend('show','Position','best');
title('Shouting kids want their balloon');
grid on;
%}


figure(5), hold on, axis equal
axis([1, Room.Width, 1, Room.Height]);
xlabel('X Position');
ylabel('Y Position');
title('Shouting kids want their balloon');

if s == 1   % only plot it once at first function call
    squarefig = zeros(1,BalArrSFM.N);
    for i = 1:BalArrSFM.N
        x_min_b = BalArrSFM.Positions(i,1) - 0.5*BalArrSFM.Edge; 
        y_min_b = BalArrSFM.Positions(i,2) - 0.5*BalArrSFM.Edge;
        x_max_b = BalArrSFM.Edge;
        y_max_b = BalArrSFM.Edge;
        squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
            'FaceColor', KidArrSFM.Color(i,:));       
    end
end

% determine # of gobjects needed for plot
%

if KidArrSFM.N < 13     % otherwise too computationally demanding
    x = ceil(KidArrSFM.N/12);   % maximum 12 each
    y = mod(KidArrSFM.N, 12);   % # > k*12
    
    for i = 1:x   
        name = "set" + num2str(i); 
        if i<x
            AL.(name) = gobjects(12,1);
            for j = 1:12 
                AL.(name)(j+12*(i-1)) = ...
                    animatedline('Color', KidArrSFM.Color(j+12*(i-1),:));
            end
    
        elseif i==x
            AL.(name) = gobjects(y,1);
            for j = 1:y 
                AL.(name)(j) = ...
                    animatedline('Color', KidArrSFM.Color(j+12*(i-1),:));
            end
        end
    end
    
    
    
    l = length(t);
    KidPosX = KidPosX(:, 1:floor(1 + l/100):end);
    KidPosY = KidPosY(:, 1:floor(1 + l/100):end);
    for i = 1:length(KidPosX)
        for j = 1:x     
            if j<x
                name = strcat('set', num2str(j));
                for k = 1:12 
                    addpoints(AL.(name)(k), KidPosX(k+12*(j-1), i), ...
                                            KidPosY(k+12*(j-1), i));
                end                                
        
            elseif j==x
                name = strcat('set', num2str(j));
                for k = 1:y 
                    addpoints(AL.(name)(k), KidPosX(k+12*(j-1), i), ...
                                            KidPosY(k+12*(j-1), i));
                end
            end
        end
        drawnow;    % limitrate;  % for faster animation
    end
end
%}



%% Writing exchanged messages into logFile for debugging
% if flag1
%     fprintf(logFile, "I'm Kid %d, i found balloon %d!\n", asd, asdf);
%     flag1 = 0;
% elseif flag2
%     fprintf(logFile, 'Collecting\n');
%     flag2 = 0;
% else
%     fprintf(logFile, 'Driving\n');
%     flag3 = 0;
% end
% 
% if norm(GCM - params.Target) < 15
%     fprintf('Yeah! Target reached')
%     return
% end





end



%%
function dydt = socialForceModel(t, y, KidArrSFM, BalArrSFM, Room)
    

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

    % Add balloons here!!!!!!!!!!!!

        
    % describes that the kid k tends to keep a situation-dependent distance
    % from the other kids j
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
    noise = (2*rand(size(KidArrSFM.Positions)) - 1)*1e-2;
    f_k = f_k0 + f_kj + f_kb + f_ka + noise;

    %% Combine velocity and position derivatives into a single vector
    dydt = [f_k(:); vel(:)];



end