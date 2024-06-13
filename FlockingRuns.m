function [t_index, DistanceTravel] = FlockingRuns(FlockingFlag, num_robots, num_obstacles)
% Summary: an utility function that runs several time according to a parameter specified in MultipleRun.m
% Description: robots try to reach their target (specified by an identity number)
% When multiple robots come close, they start to move together (flocking).
% They try to avoid other obstacles. 
% If a robot is pretty near its target it stops.

% NOTE: there can be more robots than targets and more targets than robots.
% Simulation stops when all robots have reached their target,
% or when all targets are reached.

% NOTE: FlockingFlag parameter lets user choose if flocking behavior have to be activated.
% It is possible to run the simulation with just collision avoidance behavior.
% Set FlockingFlag = false, for this second case.

%% Basic parameters 

numKid = num_robots;            
numBal = num_obstacles;

MaxNum = numKid + numBal;
Room.Width = MaxNum * 1.5;
Room.Height = MaxNum;      
DistrType = 4;
[KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room.Width, Room.Height, DistrType);

%% Flocking
 
% Simulation set-up
dt = 0.1;           % Time step
% t_end = 20;       % End time
t_index = 0;

if FlockingFlag
    k_alignment = 0.5;           % Alignment gain
    k_cohesion = 0.1;            % Cohesion gain
    k_separation = 1.5;          % Separation gain
    k_obstacle_avoidance = 1.5;  % Obstacle avoidance gain
else
    k_alignment = 0;             % Alignment gain
    k_cohesion = 0;              % Cohesion gain
    k_separation = 5.0;          % Separation gain
    k_obstacle_avoidance = 1.5;  % Obstacle avoidance gain
end

% Good values: 0.5   0.1    1.5    1.5
% Without flocking:  0      0     5.0     1.5

% Robot positions and velocities
positions = KidArray.InitPos;
separation_distance = KidArray.Radius * 3;     % Desired separation distance
velocities = 10 * KidArray.DesiredVel .* (rand(num_robots, 2) - 0.5);
v_max = 3;          % Maximum velocity
v_min = 1.5;        % Minimum velocity for robots

% Obstacles positions and velocities
obstacle_radius = BalloonArray.Edge;   % Radius of obstacles
obstacle_v_max = 0;                    % Maximum velocity for obstacles

obstacle_positions = BalloonArray.InitPos;
obstacle_velocities = obstacle_v_max * (rand(num_obstacles, 2) - 0.5);

% Array for stopping robots that reach their objcetive.
RecognitionArray = zeros(num_obstacles, 1);

% Total distance travelled 
DistanceTravel = zeros(num_robots,1);

% Set up the figure for animation
figure;
hold on;
rectangle('Position', [0 0 Room.Width Room.Height])
xlabel('X Position');
ylabel('Y Position');
title('Flocking Behavior of Multiple Robots Using Consensus Algorithm');
grid on;
axis equal;

% Create animated lines for each robot
colors = lines(num_robots);
colors_obs = lines(num_obstacles);

h = gobjects(num_robots, 1);
for i = 1:num_robots
    h(i) = animatedline('Marker', 'o', 'MarkerSize', 3, 'Color', colors(i,:));
end

obstacle_handles = gobjects(num_obstacles, 1);
for k = 1:num_obstacles
    obstacle_handles(k) = animatedline('Marker', 'square', 'MarkerSize', 2, 'Color', colors_obs(k,:));
end

disp(' ')
disp('3,2,1... Go!')
% Main simulation loop
% for t_index = 1:floor(t_end/dt)
while sum(RecognitionArray) ~= num_obstacles
    t_index = t_index + 1;

    % New velocities based on consensus algorithm
    new_velocities = zeros(num_robots, 2);
       
    % Keep track of robots arrived to their objective
    RecognitionArrayPrev = RecognitionArray;
    RecognitionArrayPrevSum = sum(RecognitionArray);
    
    for i = 1:num_robots
        pos_i = positions(i, :);
        vel_i = velocities(i, :);
        
        % Initialization
        alignment_force = [0, 0];
        cohesion_force = [0, 0];
        separation_force = [0, 0];
        obstacle_avoidance_force = [0, 0];
        num_neighbors = 0;
           
        RecognitionFlag = 0;

        for j = 1:num_robots
            if i ~= j
                pos_j = positions(j, :);
                vel_j = velocities(j, :);
                distance = norm(pos_j - pos_i);
                
                if distance < 2 * separation_distance
                    num_neighbors = num_neighbors + 1;
                    
                    % Alignment: Steer towards the average heading of local flockmates
                    alignment_force = alignment_force + vel_j;
                    
                    % Cohesion: Steer to move toward the average position of local flockmates
                    cohesion_force = cohesion_force + pos_j;
                    
                    % Separation: Steer to avoid crowding local flockmates
                    if distance < separation_distance
                        separation_force = separation_force - (pos_j - pos_i) / distance;
                    end
                end
            end
        end
        
        % Obstacle avoidance force
        for k = 1:num_obstacles
            obstacle_pos = obstacle_positions(k, :);
            distance_to_obstacle = norm(obstacle_pos - pos_i);

            if RecognitionArray(k)
                if distance_to_obstacle < 2 * obstacle_radius + separation_distance
                    obstacle_avoidance_force = obstacle_avoidance_force - (obstacle_pos - pos_i) / distance_to_obstacle;
                    
                end
            end
            
            if distance_to_obstacle < obstacle_radius + separation_distance
                obstacle_avoidance_force = obstacle_avoidance_force - (obstacle_pos - pos_i) / distance_to_obstacle;
                
            end

            if (distance_to_obstacle < obstacle_radius + separation_distance) && k == i
               RecognitionFlag = 1;
               RecognitionArray(k) = 1;
            end 
        end
          
        
        if num_neighbors > 0
            alignment_force = alignment_force / num_neighbors - vel_i;
            cohesion_force = cohesion_force / num_neighbors - pos_i;
        end
        
        % Apply the consensus algorithm
        new_velocity = vel_i + ...
                       k_alignment * alignment_force + ...
                       k_cohesion * cohesion_force + ...
                       k_separation * separation_force + ...
                       k_obstacle_avoidance * obstacle_avoidance_force;
        
        % Limit the velocity to the maximum value
        speed = norm(new_velocity);
        if speed > v_max
            new_velocity = new_velocity / speed * v_max;
        elseif speed < v_min
            new_velocity = new_velocity / speed * v_min;
        end
        
        if RecognitionFlag 
            new_velocity = 0;
        end

        new_velocities(i, :) = new_velocity;
    end
    
    % Update positions and velocities
    velocities = new_velocities;
    positions = positions + velocities * dt;

    for i=1:num_robots  
        DistanceTravel(i) = DistanceTravel(i) + norm([velocities(i,1) * dt, velocities(i,2) * dt]);
    end

    % Ensure robots stay within room limits
    safe_factor = 0.1; % 0.05;
    for i = 1:num_robots
        if positions(i, 1) < safe_factor * Room.Width
            positions(i, 1) = safe_factor * Room.Width;
            velocities(i, 1) = -velocities(i, 1);
        elseif positions(i, 1) > (1-safe_factor) * Room.Width
            positions(i, 1) = (1-safe_factor) * Room.Width;
            velocities(i, 1) = -velocities(i, 1);
        end
        
        if positions(i, 2) < safe_factor * Room.Height
            positions(i, 2) = safe_factor * Room.Height;
            velocities(i, 2) = -velocities(i, 2);
        elseif positions(i, 2) > (1-safe_factor) * Room.Height
            positions(i, 2) = (1-safe_factor) * Room.Height;
            velocities(i, 2) = -velocities(i, 2);
        end
    end

    % Update positions and velocities of obstacles
    obstacle_positions = obstacle_positions + obstacle_velocities * dt;
    
    % Ensure obstacles stay within the arena
    for k = 1:num_obstacles
        if obstacle_positions(k, 1) < safe_factor * Room.Width
            obstacle_positions(k, 1) = safe_factor * Room.Width;
            obstacle_velocities(k, 1) = -obstacle_velocities(k, 1);
        elseif obstacle_positions(k, 1) > (1-safe_factor) * Room.Width
            obstacle_positions(k, 1) = (1-safe_factor) * Room.Width;
            obstacle_velocities(k, 1) = -obstacle_velocities(k, 1);
        end
        
        if obstacle_positions(k, 2) < safe_factor * Room.Width
            obstacle_positions(k, 2) = safe_factor * Room.Width;
            obstacle_velocities(k, 2) = -obstacle_velocities(k, 2);
        elseif obstacle_positions(k, 2) > (1-safe_factor) * Room.Width
            obstacle_positions(k, 2) = (1-safe_factor) * Room.Width;
            obstacle_velocities(k, 2) = -obstacle_velocities(k, 2);
        end
    end

    % Update the animated lines
    for i = 1:num_robots
        clearpoints(h(i));
        addpoints(h(i), positions(i, 1), positions(i, 2));
        % drawnow limitrate
        xlim([-0.1*Room.Width 1.1*Room.Width])
        ylim([-0.1*Room.Height, 1.1*Room.Height])
    end
    
    % Plot robots
    for i = 1:num_robots
         x_min = positions(i,1) - KidArray.Radius;
         y_min = positions(i,2) - KidArray.Radius;
         radius_cur = KidArray.Radius;
         KidArray.circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
             'Curvature',[1 1], 'FaceColor',KidArray.Color(i,:));
         KidArray.text(i) = text(positions(i,1), positions(i,2), num2str(KidArray.ID(i)), ...
             'HorizontalAlignment', 'center', 'Color','k', 'FontSize', KidArray.Radius*15);
    end

    % Update the animated lines
    for k = 1:num_obstacles
        clearpoints(obstacle_handles(k));
        addpoints(obstacle_handles(k), obstacle_positions(k, 1), obstacle_positions(k, 2));
        % drawnow limitrate
        xlim([-0.1*Room.Width 1.1*Room.Width])
        ylim([-0.1*Room.Height, 1.1*Room.Height])
    end
    
    % Plot obstacles
    for k = 1:num_obstacles
         x_min_b = obstacle_positions(k,1) - 0.5*BalloonArray.Edge; 
         y_min_b = obstacle_positions(k,2) - 0.5*BalloonArray.Edge;
         x_max_b = BalloonArray.Edge;
         y_max_b = BalloonArray.Edge;
         if(k > height(KidArray.Color))
             bcolor = rand(1,3);
             BalloonArray.squarefig(k) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
             'FaceColor',bcolor);
         else 
             BalloonArray.squarefig(k) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
             'FaceColor',KidArray.Color(k,:));
         end
     
         BalloonArray.plotBalID(k) = text(obstacle_positions(k,1), obstacle_positions(k,2), num2str(BalloonArray.ID(k)), ...
             'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalloonArray.Edge*10);
    end
   
    ind = text(-0.5, -0.5, num2str(t_index));
    % Pause to update the plot
    pause(0.01);

    % Delete previous figure elements
    delete(KidArray.circlefig)
    delete(KidArray.text)
    delete(BalloonArray.squarefig)
    delete(BalloonArray.plotBalID)
    delete (ind)

    % Update recognition situation
    if sum(RecognitionArray) > RecognitionArrayPrevSum
        ind = find(RecognitionArray - RecognitionArrayPrev)';
        if length(ind) < 2
            disp([ 'Kid number ' num2str(ind) ' has taken his balloon!'])
        else
            disp([ 'Kids number ' num2str(ind') ' have taken their balloons!'])
        end
        disp([ 'Balloons overview: ' num2str(RecognitionArray') ]);
    end

end

% Keep the final plot visible
hold off;

% Display useful information
disp(' ');
disp([ 'Total number of steps: ' num2str(t_index) ]);
disp([ 'Distance Travelled by each robot: ' num2str(DistanceTravel') ]);

% Final position plot
for i = 1:num_robots
         x_min = positions(i,1) - KidArray.Radius;
         y_min = positions(i,2) - KidArray.Radius;
         radius_cur = KidArray.Radius;
         KidArray.circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
             'Curvature',[1 1], 'FaceColor',KidArray.Color(i,:));
         KidArray.text(i) = text(positions(i,1), positions(i,2), num2str(KidArray.ID(i)), ...
             'HorizontalAlignment', 'center', 'Color','k', 'FontSize', KidArray.Radius*15);
end

for k = 1:num_obstacles
         x_min_b = obstacle_positions(k,1) - 0.5*BalloonArray.Edge; 
         y_min_b = obstacle_positions(k,2) - 0.5*BalloonArray.Edge;
         x_max_b = BalloonArray.Edge;
         y_max_b = BalloonArray.Edge;
         if(k > height(KidArray.Color))
             bcolor = rand(1,3);
             BalloonArray.squarefig(k) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
             'FaceColor',bcolor);
         else 
             BalloonArray.squarefig(k) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
             'FaceColor',KidArray.Color(k,:));
         end
     
         BalloonArray.plotBalID(k) = text(obstacle_positions(k,1), obstacle_positions(k,2), num2str(BalloonArray.ID(k)), ...
             'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalloonArray.Edge*10);
end

end