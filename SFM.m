function [KidPos, KidVel] = SFM(kid, params)%, logFile)
% Implementation of the Social Force Model acoording to the paper 
% "Parameter Calibration of a Social Force Model for the Crowd-Induced 
% Vibrations of Footbridges" by Elisa Bassoli and Loris Vincenzi

% alpha from the paper becomes k
% beta from the paper becomes j
    
%% Counter for number of function calls (for debugging)
persistent s 

if s > params.numSteps
    s = 0;
elseif isempty(s)
    s = 0;
end
s = s + 1;

%% Kid movement
%% Equation 4, Driving term f_k0
% intention of each kid to walk with a desired speed v_k0 towards its
% destination and the fact that deviations of the actual velocity v_k
% from the desired velocity v0 are corrected within the relaxation time

% Desired direction of motion (Equation 5)
e_k = normalize(kid.Destination - kid.Positions, 'norm', 2);
    % instead of final destination this should be the 'next edge' on an
    % imaginary polygon => how to implement this? is it even necessary?

tau = 0.5;  % relaxation time (see table 1)
%v_k = ;% to be obtained from eq 2
f_k0 = 1/tau*(kid.DesiredVel.*e_k - kid.Velocities);  

% seems too small to me in comparison to the other forces

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
            d_kj = kid.Positions(k,:) - kid.Positions(j,:);    % distance between CoM
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
d_kb = zeros(2);      % initialization, 1 for x, 2 for y
A = 0.5;                % repulsive interaction strength, see table 1
B = 0.1;                % repulsive interaction range, see table 1

for k=1:kid.N
    
        % simple room with 4 walls => make more general later
        % here it's not yet exactly like in the paper, because the
        % paper sums the influence of all walls, while this code
        % only takes the closest wall in every direction into account
        d_kb(1,1) = min([abs(kid.Positions(k,1) - params.roomWidth), ...
                       abs(kid.Positions(k,1) - 0)]);   % distance in x
        d_kb(2,2) = min([abs(kid.Positions(k,2) - params.roomLength), ...
                       abs(kid.Positions(k,2) - 0)]);     % distance in y

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
            d_kj = kid.Positions(k,:) - kid.Positions(j,:);    % distance between CoM
            n_kj = normalize(d_kj, 'norm', 2);
            f_ka(k,:) = f_ka(k,:) + A * exp((r_kj - norm(d_kj))/B) * n_kj * ...
                        (lambda_k + (1 - lambda_k)*0.5*(1 - n_kj*e_k(k,:)') + 1);
        end
    end
end


%% Define the system of ODEs (Equations 1 to 3)
% "Euler-Integration"
dt = 0.1;
f_k = (f_k0 + f_kj + f_kb + f_ka);
KidVel = kid.Velocities + f_k*dt;
% dv/dt = f_k0 + f_kj + f_kb + f_ka (Eq. 3)

KidPos = kid.Positions + KidVel*dt;   % dx/dt = v



%%
% % Weights for each direction vector
% weight_attraction_to_GCM = 1.05;
% weight_repulsion_from_neighbor = 2;
% weight_repulsion_from_shepherd = 1;
% weight_proceeding_as_previously = 0.5;
% noise_strength = 0.05;
% 
% % Parameters
% r_shepherd = 65; % Radius of influence for the shepherd
% r_neighbor = 2;  % Radius of influence for neighbor animals
% step_size_animal = zeros(1,kid.N); % Grazing (no movement) by default
% 
% % Compute the Global Center of Mass (GCM) of the herd
% GCM = mean(kid.Positions, 2);
% 
% % Initialize matrices for direction vectors
% repulsion_from_neighbor = zeros(2, kid.N); % Repulsion from nearest neighbor
% repulsion_from_shepherd = zeros(2, kid.N); % Repulsion from shepherd
% 
% % Calculate attraction vector to GCM 
% % and normalize to length 1
% attraction_to_GCM = normalize(GCM - kid.Positions, 'norm', 2); % vector
% 
% 
% % --> later with LCM !!!
% % n closest neighbors form the LCM
% n = round(0.9 * kid.N);
% 
% for i = 1:kid.N    
% 
%     % Calculate repulsion from nearest neighbor and find its index
%     d_next_neighbor = sqrt(sum((kid.Positions - ...
%                                 kid.Positions(:, i)).^2, 1));
%     d_next_neighbor(i) = inf; % Exclude the animal itself
% 
%     % Implementation of formula (4.1) in [Strömbom et al]
%     neighborIndeces = find(d_next_neighbor <= r_neighbor);
%     l = length(neighborIndeces);
%     memry1 = zeros(2,l);
%     for j = 1:l
%         memry1(:,j) = kid.Positions(:,i) - (kid.Positions(:,neighborIndeces(j)));
%     end
%     repulsion_from_neighbor(:,i) = sum(normalize(memry1, 'norm',2),2);
% 
%     if ~any(neighborIndeces)
%         % if no animal near, the zero should remain, also in the above code
%         % still we need
%     end
% 
%     % Calculate repulsion from shepherds (if within the influence radius)
%     % analogous implementation for more than one shepherd robot
%     d_to_shepherd = sqrt(sum((Shepherd.Positions - kid.Positions(:, i)).^2, 1));
%     shepherds_within_range = find(d_to_shepherd <= r_shepherd); % indices of shepherds in range     
%     m = length(shepherds_within_range);
%     memry2 = zeros(2,m);
%     for k = 1:m
%         memry2(:,k) = kid.Positions(:,i) - ...
%                 (Shepherd.Positions(:,shepherds_within_range(k)));
%     end
%     repulsion_from_shepherd(:,i) = sum(normalize(memry2, 'norm',2),2);
% 
%     % Get animal i from grazing to flight mode only if shepherd is near
%     if any(shepherds_within_range)
%         step_size_animal(i) = 1;
%     end
% 
% 
% end
% 
% 
% % Apply weights to direction vectors
% weighted_continuation = weight_proceeding_as_previously * ...
%                     normalize(kid.Direction, 'norm', 2);
% weighted_attraction = weight_attraction_to_GCM * attraction_to_GCM;
% weighted_repulsion_neighbor = weight_repulsion_from_neighbor * ...
%                     normalize(repulsion_from_neighbor, 'norm', 2);
% weighted_repulsion_shepherd = weight_repulsion_from_shepherd * ...
%                     normalize(repulsion_from_shepherd, 'norm', 2);
% 
% % Set NaN from normalization of 0 back to zero
% weighted_continuation(isnan(weighted_continuation)) = 0;
% weighted_repulsion_neighbor(isnan(weighted_repulsion_neighbor)) = 0;
% weighted_repulsion_shepherd(isnan(weighted_repulsion_shepherd)) = 0;
% 
% % Add a noise term to introduce randomness
% noise = noise_strength * randn(2, kid.N);
% 
% % Calculate the resulting direction vector for each animal
% KidDirec = weighted_continuation + ...
%               weighted_attraction + ...
%               weighted_repulsion_neighbor + ...
%               weighted_repulsion_shepherd; % formula (4.2)
% KidDirec = normalize(KidDirec, 'norm', 2);
% 
% % Calculate displacement for animals and update positions
% AnimalDispl = step_size_animal .* KidDirec + noise;  % moved noise to 
% %               here to always have minimal movement even while grazing
% KidPos = kid.Positions + AnimalDispl;
% 
% 
% 
% % Ensure that the positions remain within the field boundaries
% KidPos(1, :) = max(1, min(KidPos(1, :), params.fieldWidth));
% KidPos(2, :) = max(1, min(KidPos(2, :), params.fieldLength));
% 
% 
% %% Shepherd movement
% 
% % Calculate displacement for shepherds and update positions
% step_size_shepherd = 1.5; % Desired step size for shepherds
% ShepherdDirec = zeros(2,Shepherd.N);
% 
% % Definition of furthest allowed distance from GCM as f(Animal.N)
% d_furthest = 55;%1.2 * r_shepherd;    % must be > r_shepherd => why??
% 
% % Compute the Global Center of Mass (GCM) of the herd
% % Compute Animals distance from GCM
% GCM = mean(kid.Positions, 2);
% d_animals_GCM = zeros(1, kid.N);
% d_animals_target = zeros(1, kid.N);
% 
% flag1 = 0;
% flag2 = 0;
% flag3 = 0;
% 
% % Calculate distances of all animals from GCM and target
% for i = 1:kid.N
%     d_animals_GCM(i) = norm(kid.Positions(:, i) - GCM);
%     d_animals_target(i) = norm(kid.Positions(:, i) - params.Target);
% end
% 
% for i = 1:Shepherd.N
% 
%     % Calculate all distances between the animals to the robot i
%     distances_to_agents = zeros(Shepherd.N, kid.N);
% 
%  % the distance i've already implemented above => reuse later when improving
%  % code!!
%     % Calculate distances to all animals from shepherd i
%     for j = 1:kid.N
%         distances_to_agents(Shepherd.N, j) = norm(kid.Positions(:, j) - ...
%                             Shepherd.Positions(:, i));
%     end
% 
% 
%     if any(d_animals_GCM > d_furthest)        
%         % Animal too far off, Calculate the collecting position
%         % Find the maximum value of the furthest animal and its index
% 
%     % Improvement:
%     % Aim for the animal that is furthest from GCM and also from target
%         Indx_Animals_maxGCM = find(d_animals_GCM >= d_furthest);        
%         [maxValues,Indx_Animals_maxTarget] = maxk(d_animals_target, 10);
% 
%         Indx_Animal_Collect = intersect(Indx_Animals_maxGCM,...
%                                         Indx_Animals_maxTarget);
%         if isempty(Indx_Animal_Collect)
%             Indx_Animal_Collect = Indx_Animals_maxTarget;
%         end
%         LCM = mean(kid.Positions(:,Indx_Animal_Collect), 2);
% 
%         PosColl = LCM + normalize(LCM - GCM,'norm',2) * r_neighbor;
% 
%         flag2 = 1;
% 
%         % Determine Direction
%         ShepherdDirec = (PosColl - Shepherd.Positions(:, i))/...
%                         norm(PosColl - Shepherd.Positions(:, i));
%         ShepherdDirec = normalize(ShepherdDirec, 'norm', 2);
% 
%     elseif any(distances_to_agents <= 20 * r_neighbor)
%         % If the shepherd is within 3*r_neighbor from any agent, 
%         % it's speed should remain zero
% 
%         flag1 = 1;
% 
%     else
%         PosDrive = GCM - normalize(GCM - params.Target,'norm',2) * ...
%                                         r_neighbor * sqrt(kid.N);
%         % All animals in flock, let them graze and then drive to target
%         %pause(1)    % tbd later
% 
%         % Implement here a logic that takes shepherd position in relation
%         % to GCM and target into account
%         % u = [GCM - Shepherd.Positions; 0];  
%         % v = [params.Target - Shepherd.Positions; 0];
%         % w = GCM - params.Target;
%         % angle = atan2d(norm(cross(u,v)), dot(u,v));
%         % sidecheck = cross(v,w); % if <0 Sh. left of w, if >0 right of w
%         % 
%         % if (angle < 90) & (norm(v) > norm(w))
%         %     % final case we want to reach
%         %     PosDrive = GCM - normalize(w,'norm',2) * ...
%         %                                 r_neighbor * sqrt(Animal.N);
%         % 
%         % 
%         % elseif (angle < 90) & (norm(v) < norm(w))
%         %     PosDrive = 
%         % 
%         % elseif (angle > 90) & (norm(v) < norm(w))
%         %     PosDrive = 
%         % 
%         % 
%         % 
%         % end
% 
%         flag3 = 1;
% 
%         % Determine Direction
%         ShepherdDirec = (PosDrive - Shepherd.Positions(:, i))/...
%                         norm(PosDrive - Shepherd.Positions(:, i));
%         ShepherdDirec = normalize(ShepherdDirec, 'norm', 2);
%     end
% 
%     % Calculate displacement for shepherd and update positions
%     ShepherdDispl = step_size_shepherd .* ShepherdDirec;
%     ShepherdPos = Shepherd.Positions + ShepherdDispl;
% end
% 
% % Ensure that the positions remain within the field boundaries
% ShepherdPos(1, :) = max(1, min(ShepherdPos(1, :), params.fieldWidth));
% ShepherdPos(2, :) = max(1, min(ShepherdPos(2, :), params.fieldLength));
% 
% 
% if flag1
%     fprintf(logFile, 'Step %d: Robot too close\n', s);
%     flag1 = 0;
% elseif flag2
%     fprintf(logFile, 'Step %d: Collecting\n', s);
%     flag2 = 0;
% else
%     fprintf(logFile, 'Step %d: Driving\n', s);
%     flag3 = 0;
% end
% 
% 
% if norm(GCM - params.Target) < 15
%     fprintf('Yeah! Target reached')
%     return
% end
% 
% 
% 
% % Hardcode shepherd movement
% % if s<2
% %     ShepherdPos = Shepherd.Positions + [-1;0];  % apparently i have [y;x]....
% % elseif s>=2 & s<160
% %     ShepherdPos = Shepherd.Positions + [0;1];
% % elseif s>=160 & s<310
% %     ShepherdPos = Shepherd.Positions + [0.6;0];
% % elseif s>=310
% %     ShepherdPos = Shepherd.Positions + [0;-0.6];
% % end

end