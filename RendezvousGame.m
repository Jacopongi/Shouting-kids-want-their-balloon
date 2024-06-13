function [] = RendezvousGame 
% Summary: simulate a multiple robot rendezvous behavior
% Description: given random positions to multiple robots, 
% firstly they try to reach the center of the room 
% according to a consensus algorithm.
% Then user can choose the next position of rendezvous 
% by picking it in a pre-defined area.
% Robots will move to that new target position
% and evenly dispose around it.

% NOTE: user can select three different target positions.

% Parameters
numRobots = 5;      % Number of robots
dim = 2;            % Dimension (2D case)
alpha = 0.1;        % Step size for the consensus update
epsilon = 1e-3;     % Convergence threshold
maxIterations = 1000;   % Maximum number of iterations
minDistance = 0.1;      % Minimum distance between robots

% Initial positions of the robots 
positions = rand(numRobots, dim);

% Initial position of target
targetPoint = [0.5, 0.5];
positions = [positions; targetPoint];

% Adjacency matrix (all robots can communicate with each other)
adjMatrix = ones(numRobots + 1) - eye(numRobots + 1);

% Simulation
iteration = 0;
previousiter = 0;

% Define the boundaries of the allowed area
x_min = 0.2;
x_max = 0.8;
y_min = 0.2;
y_max = 0.8;

stop = 0;
attempt = 0;

% Plot positions
figure
hold on;
% Plot the target 
hTarget = plot(targetPoint(1), targetPoint(2), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
% Plot the robots
hRobots = plot(positions(1:end-1, 1), positions(1:end-1, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
title(sprintf('Iteration %d', iteration));
xlim([0 1]);
ylim([0 1]);
drawnow;

disp(' ')
disp('WELCOME TO RENDEZVOUS GAME')
disp('Here you can try how rendezvous consensus algorithm works.')
disp('After the first run, you can pick your favorite position inside the room...')
disp('... and robots will converge there!')
disp(' ')
disp(['Seems funny, doesn''' 'it? It is. But pay attention you have just 3 picks.'])
disp([' Are you ready? Let''' 's go!'])

while stop ~= 1

    converged = false;
    while ~converged && iteration < maxIterations
       
        % Store previous positions
        positions(end,:) = targetPoint;
        prevPositions = positions;
        
        % Update each robot's position to move towards the target 
        for i = 1:numRobots
            % Attraction force towards the target
            attractionForce = alpha * (positions(end, :) - positions(i, :));
            
            % Repulsion forces between neighboring robots and the target
            repulsionForce = zeros(1, dim);
            for j = 1:numRobots + 1
                if j ~= i
                    d = positions(i, :) - positions(j, :);
                    if norm(d) < minDistance
                        repulsionForce = repulsionForce + d / norm(d) * (minDistance - norm(d));
                    end
                end
            end
            
            % Update position
            positions(i, :) = positions(i, :) + attractionForce + repulsionForce;
        end
        
        % Check for convergence
        if max(max(abs(positions(1:end-1, :) - prevPositions(1:end-1, :)))) < epsilon
            converged = true;
        end
        
        iteration = iteration + 1;
        
        % Plot
        %clf;
        figure(8888)
        delete(hTarget);
        delete(hRobots);
        hold on;
        % Plot the target 
        hTarget = plot(targetPoint(1), targetPoint(2), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
        % Plot the robots
        hRobots = plot(positions(1:end-1, 1), positions(1:end-1, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        title(sprintf('Iteration %d', iteration));
        xlim([0 1]);
        ylim([0 1]);
        drawnow;

        % Pause for visualization
        pause(0.1); 

        if converged
            disp(' ')
            disp('Running, running...')
            disp('Boss, we are all arrived!')
            fprintf('We have taken %d steps.\n', (iteration-previousiter));
            disp(' ')

            previousiter = iteration;
            attempt = attempt + 1;
            if attempt == 4
                
                thanks = text(0.5, 0.95, 'Thanks for playing', ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
                theend = text(0.5, 0.90, 'THE END', ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);

                for i=0.05:0.05:0.80
                    delete(thanks)
                    delete(theend)
                    thanks = text(0.5, 0.95-i, 'Thanks for playing', ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
                    theend = text(0.5, 0.90-i, 'THE END', ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
                    pause(0.1)
                end
                
                disp('It seems that you finished your picks!')
                disp('Thanks for playing. Bye!')

                stop = 1;
                break
            end

            % Plot information lines and allowed area
            info = text(0.5, y_min/2, 'Hint: pick a point inside allowed area', ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
            rematt = text(0.5, 0.9, ['Remaining pick: ' num2str(4-attempt)], ...
                    'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
            allowarea = rectangle('Position', [x_min, y_min, x_max - x_min, y_max - y_min], 'EdgeColor', 'r', ...
                      'LineWidth', 2, 'LineStyle','-');

            while true
                disp('Where do we go next?')
                [x, y] = ginput(1);             % Update target point based on user input
                
                if x >= x_min && x <= x_max && y >= y_min && y <= y_max
                    targetPoint = [x, y];       % Store new coordinates in targetPoint
                    disp(['Alright. We go to [' num2str(targetPoint(1), 2) ' ' num2str(targetPoint(2), 2) ']!'])
                    delete(info)
                    delete(rematt)
                    delete(allowarea)
                    break;                      % Exit the loop if the point is within the specified area
                else
                    warn = text(0.5, 0.5, 'Hey, I said INSIDE allowed area!', ...
                         'HorizontalAlignment', 'center', 'Color','k', 'FontSize', 10);
                    disp('Point outside the specified area. Please select again.');
                    pause(2)
                end
                delete(warn)
            end

            set(hTarget, 'XData', x, 'YData', y);
        end
    
    end

end

end

