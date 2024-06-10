function [Test] = TestEstimatePositionNoCons(Sensor, Room, Test, gridnum)
% Summary: estimate several reference positions for testing purpose.
% Description: Reference positions are obtained with a grid. 
% The nearest sensor to considered position makes the estimate.
% Uncertainties on estimation are imported from TestEstimatePosition. 

% Several plots are available but left commented for visualization ease.

% NOTE: 'gridnum' represents one dimension of the grid. Grid is symmetric.


% Positions instantiation
EstimatedPos = zeros(gridnum^2, 2);
Env.RefPosition = zeros(1,2);
UsedPos = zeros(gridnum^2, 2);
Env.Positions = Test.ReferencePos;

% Sensors fields
Sensor.PosMeas = zeros(Sensor.Num, 2);
Sensor.Detect = zeros(Sensor.Num, 1);

% Errors 
err_x = zeros(1, gridnum^2);
err_y = zeros(1, gridnum^2);

%% Simulation
for k = 1:gridnum^2

    Env.RefPosition = Env.Positions(k,:);
    Sensor.PosMeas = zeros(Sensor.Num, 2);
    Sensor.Detect = zeros(Sensor.Num, 1);
    DistSensPos = zeros(Sensor.Num, 1);

    for i = 1:Sensor.Num
        DistSensPos(i) = norm(Sensor.Position(i,:) - Env.RefPosition);
          % Distance Check and Measurement
          if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) && (Sensor.Detect(i) == 0) 
                Sensor.PosMeas(i,:) = Env.RefPosition + Test.collect(i,:,k);
                Sensor.Detect(i) = 1;
          end
    end

    [MinDist, MinIndex] = min(DistSensPos);

    EstimatedPos(k,1) = Sensor.PosMeas(MinIndex,1); 
    EstimatedPos(k,2) = Sensor.PosMeas(MinIndex,2);

    UsedPos(k,:) = Env.RefPosition;
  
    err_x(k) = EstimatedPos(k,1) - Env.RefPosition(1);
    err_y(k) = EstimatedPos(k,2) - Env.RefPosition(2);

    % disp(['Analysed Position: ', num2str(k), ' - Sensor number: ', num2str(MinIndex), ' Error: ', num2str(err_x(k), 3), ' ' num2str(err_y(k),3) ]);

    % Sensors with single position estimate (actual position and estimated one) 
    %{
    figure;
    axis equal;
    axis([-0.2*Room.Width 1.2*Room.Width -0.2*Room.Height 1.2*Room.Height])
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Single Position Estimation','FontSize',14)
    hold on;
    
    rectangle('Position', [0, 0, Room.Width, Room.Height], 'LineWidth', 2);
    plot(Env.RefPosition(1), Env.RefPosition(2), '--o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(EstimatedPos(k,1), EstimatedPos(k,2), '*', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    
    for i = 1:Sensor.Num 
            plot(Sensor.Position(i,1), Sensor.Position(i,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            text(Sensor.Position(i,1), Sensor.Position(i,2), num2str(i), 'HorizontalAlignment', 'center');
            if (Sensor.Detect(i))
                rectangle('Position', [Sensor.Position(i,1) - Sensor.Range, Sensor.Position(i,2) - Sensor.Range, ...
                                       Sensor.Range * 2, Sensor.Range * 2], ...
                          'Curvature',[1, 1], 'LineStyle','--');
                plot([Sensor.Position(i,1), Env.RefPosition(1)], [Sensor.Position(i,2), Env.RefPosition(2)], 'LineStyle','-.');
            end
    end
    hold off
    %}
end

% Save estimates and errors
Test.EstimatedPosNC = EstimatedPos;
Test.err_x_NC = err_x;
Test.err_y_NC = err_y;

%% Errors analysis

%{
mean_err_x = mean(err_x);
mean_err_y = mean(err_y);

std_err_x = std(err_x);
std_err_y = std(err_y);

max_err_x = max(err_x);
max_err_y = max(err_y);

rmse_x = rmse(Env.Positions(:,1), EstimatedPos(:,1));
rmse_y = rmse(Env.Positions(:,2), EstimatedPos(:,2));

disp('-------------------------')
disp('Errors of Position Estimation without Consensus Algorithm')
disp(['The mean error on x coordinate is: ', num2str(mean_err_x, 3), ' m']);
disp(['The mean error on y coordinate is: ', num2str(mean_err_y, 3), ' m']);

disp(['The standard deviation on x coordinate is: ', num2str(std_err_x, 3), ' m']);
disp(['The standard deviation on y coordinate is: ', num2str(std_err_y, 3), ' m']);
 
disp(['The maximum error on x coordinate is: ', num2str(max_err_x, 3), ' m']);
disp(['The maximum error on y coordinate is: ', num2str(max_err_y, 3), ' m']);

disp(['The root mean squared error on x coordinate is: ', num2str(rmse_x, 3), ' m']);
disp(['The root mean squared error on y coordinate is: ', num2str(rmse_y, 3), ' m']);

disp('-------------------------')


% Plot errors in x-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Error of Position Estimation in X-coordinate (No consensus)','FontSize',14)
hold on;
plot(1:gridnum^2, err_x);
% xticks(1:gridnum^2);
hold off

% Plot errors in y-position estimation
figure;
box on
grid 
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Error of Position Estimation in Y-coordinate (No consensus)','FontSize',14)
hold on;
plot(1:gridnum^2, err_y);
% xticks(1:gridnum^2)
hold off

%}

%% Sensors and all positions (actual and estimated ones)

% figure;
% axis equal;
% axis([-0.2*Room.Width 1.2*Room.Width -0.2*Room.Height 1.2*Room.Height])
% box on
% xlabel('x','FontSize',16)
% ylabel('y','FontSize',16)
% title('Sensors and Positions','FontSize',14)
% hold on;
% 
% rectangle('Position', [0, 0, Room.Width, Room.Height], 'LineWidth', 2);
% 
% for k=1:gridnum^2
% 
%     Env.RefPosition = UsedPos(k,:);
%     plot(Env.RefPosition(1), Env.RefPosition(2), '+', 'MarkerSize', 10, 'linewidth', 0.5, 'DisplayName','Actual Position');
%     plot(EstimatedPos(k,1), EstimatedPos(k,2), 'x', 'MarkerSize', 10 ,'linewidth', 0.5, 'DisplayName','Estimated Position');
% 
%     for i = 1:Sensor.Num 
%             plot(Sensor.Position(i,1), Sensor.Position(i,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName','Sensor');
%             legend('AutoUpdate','off')
%             text(Sensor.Position(i,1), Sensor.Position(i,2), num2str(i), 'HorizontalAlignment', 'center');
%     end
% 
%     plot([EstimatedPos(k,1), Env.RefPosition(1)], [EstimatedPos(k,2), Env.RefPosition(2)], 'LineStyle','-.');
% end
% 
% hold off

end
