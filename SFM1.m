function [KidPos, KidVel] = SFM1(kid, params)
% Implementation of the Social Force Model acoording to the paper 
% "Parameter Calibration of a Social Force Model for the Crowd-Induced 
% Vibrations of Footbridges" by Elisa Bassoli and Loris Vincenzi

% alpha from the paper becomes k
% beta from the paper becomes j
    
%% Counter for number of function calls (for debugging)
% persistent s 
% 
% if s > params.numSteps
%     s = 0;
% elseif isempty(s)
%     s = 0;
% end
% s = s + 1;

%% Kid movement

% Combine initial positions and velocities into a single vector
initCond = [kid.Velocities(:); kid.Positions(:)];

% Set simulation time
tSpan = [0, 20];    % for example

options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
tic
[t, y] = ode45(@(t,y) socialForceModel(t,y,kid,params), tSpan, initCond, options);
toc
% Extract results
KidVelX = reshape(y(:,         1:   kid.N)', kid.N, []);
KidVelY = reshape(y(:,   kid.N+1: 2*kid.N)', kid.N, []);
KidPosX = reshape(y(:, 2*kid.N+1: 3*kid.N)', kid.N, []);
KidPosY = reshape(y(:, 3*kid.N+1:     end)', kid.N, []);

KidPos = [KidPosX, KidPosY];
KidVel = [KidVelX, KidVelY];

% Plot the results
figure(1);
for i = 1:kid.N
    plot(KidPosX(i,:), KidPosY(i,:), 'DisplayName', ['Kid ', num2str(i)]);
    hold on;    
end
axis equal;
plot(kid.Destination(:,1), kid.Destination(:,2), 'g*')  % balloons
plot(kid.Positions(:,1), kid.Positions(:,2), 'ro')      % starting pos
axis([1, params.roomWidth, 1, params.roomLength]);
xlabel('X Position');
ylabel('Y Position');
%legend('show','Position','best');
title('Shouting kids want their balloon');
grid on;

figure(2), clf, hold on
AL = gobjects(kid.N, 1);
for i = 1:kid.N
    AL(i) = animatedline('Color', rand(1, 3));
end
axis equal
axis([1, params.roomWidth, 1, params.roomLength]);
xlabel('X Position');
ylabel('Y Position');
title('Shouting kids want their balloon');
plot(kid.Destination(:,1), kid.Destination(:,2), 'g*')
hold on
for l = 1:length(t)
    for i = 1:kid.N
        % Add a point to the animated line for each kid
        addpoints(AL(i), KidPosX(i, l), KidPosY(i, l));
    end
    drawnow;        
end




end



function dydt = socialForceModel(t, y, kid, params)
    

    % Extract velocities and positions from the state vector
    vel = reshape(y(1:2*kid.N), kid.N, []);
    pos = reshape(y(2*kid.N+1:end), kid.N, []);

    %% Equation 4, Driving term f_k0
    % intention of each kid to walk with a desired speed v_k0 towards its
    % destination and the fact that deviations of the actual velocity v_k
    % from the desired velocity v0 are corrected within the relaxation time
    
    % Desired direction of motion (Equation 5)
    e_k = normalize(kid.Destination - pos, 'norm', 2);
        % instead of final destination this should be the 'next edge' on an
        % imaginary polygon => how to implement this? is it even necessary?
    
    tau = 0.5;  % relaxation time (see table 1)
    %v_k = ;% to be obtained from eq 2
    f_k0 = 1/tau*(kid.DesiredVel.*e_k - vel);  
    
    
    %% Equation 6, Repulsive force f_kj
        
    % describes that the kid k tends to keep a situation-dependent distance
    % from the other kids j
    f_kj = zeros(kid.N,2);
    
    A = 0.5;            % repulsive interaction strength, see table 1
    B = 0.1;            % repulsive interaction range, see table 1
    r_kj = 2*kid.r;     % sum of their radii
    lambda_k = 0.5;     % anisotropic factor, see figure 1
    
    for k=1:kid.N
        for j=1:kid.N
            if j~=k
                d_kj = pos(k,:) - pos(j,:);    % distance between CoM
                n_kj = normalize(d_kj, 'norm', 2);
                f_kj(k,:) = f_kj(k,:) + A * exp((r_kj - norm(d_kj))/B) * n_kj * ...
                            (lambda_k + (1 - lambda_k)*0.5*(1 - n_kj*e_k(k,:)') + 1);
            end
        end
    end
        
    
    %% Equation 7, Repulsion from borders f_kb
    
    % intention to keep a certain distance from borders. The repulsive effect
    % of borders is similar to the repulsion among pedestrians except for the
    % anisotropic behavior.
    f_kb = zeros(kid.N,2);  % force in x and y direction on kid k (Nx2)
    d_kb = zeros(2);        % initialization, 1 for x, 2 for y
    A = 0.5;                % repulsive interaction strength, see table 1
    B = 0.1;                % repulsive interaction range, see table 1
    
    for k=1:kid.N
        
            % simple room with 4 walls => make more general later
            % here it's not yet exactly like in the paper, because the
            % paper sums the influence of all walls, while this code
            % only takes the closest wall in every direction into account
            d_kb(1,1) = min([abs(pos(k,1) - params.roomWidth), ...
                             abs(pos(k,1) - 0)]);   % distance in x
            d_kb(2,2) = min([abs(pos(k,2) - params.roomLength), ...
                             abs(pos(k,2) - 0)]);     % distance in y
    
        for b = 1:2
            n_kb = normalize(d_kb(b,:), 'norm', 2);
            f_kb(k,:) = f_kb(k,:) + A*exp((kid.r - norm(d_kb(b,:)))/B) * n_kb;
           
        end
    end
        
    
    %% Equation 6a), Attraction force f_ka
        
    % "These attractive forces can be modeled accordingly to the
    % repulsive forces among pedestrians of Eq. 6 with longer
    % interaction range and negative interaction strength."
    f_ka = zeros(kid.N,2);
    
    A = -0.5;        % attractive interaction strength, sign inverted
    B = 0.5;         % repulsive interaction range, >0.1 => modifiy and see what works
    r_kj = 2*kid.r;     % sum of their radii
    lambda_k = 0.5;     % anisotropic factor, see figure 1
    
    for k=1:kid.N
        for j=1:kid.N
            if j~=k
                d_kj = pos(k,:) - pos(j,:);    % distance between CoM
                n_kj = normalize(d_kj, 'norm', 2);
                f_ka(k,:) = f_ka(k,:) + A * exp((r_kj - norm(d_kj))/B) * n_kj * ...
                            (lambda_k + (1 - lambda_k)*0.5*(1 - n_kj*e_k(k,:)') + 1);
            end
        end
    end
    
    
    %% Define the system of ODEs (Equations 1 to 3)     
    f_k = f_k0 + f_kj + f_kb + f_ka;

    %% Combine velocity and position derivatives into a single vector
    dydt = [f_k(:); vel(:)];



end