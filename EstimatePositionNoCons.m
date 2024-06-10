function [EstimatedPos] = EstimatePositionNoCons(Array, Sensor, Room)
% Summary: estimate agents position without Consensus Algorithm.
% Description: the nearest sensor to considered position makes the estimate.
% Uncertainties on estimation come from measuring system (sensors) and
% enviroment (added random noise). 


% Position instantiation
EstimatedPos = zeros(Array.N, 2);
Env.RefPosition = zeros(1,2);
UsedPos = zeros(Array.N, 2);

% Adding External noise (independent sources affecting each dimension)
Env.NoiseCov = 0.5 * randn(2);         % Env.NoiseCov = diag(rand(2,1));           
% Alternative: Env.NoiseCov =  random('Poisson', 1, 2)*0.1;
Env.NoiseCov = Env.NoiseCov'*Env.NoiseCov;       
Sensor.PosCov = Sensor.PosCov + Env.NoiseCov;    % Remember: Covariance_Total = Covariance_Sensor + Covariance_Noise 

% Sensor fields
Sensor.PosMeas = zeros(Sensor.Num, 2);
Sensor.Detect = zeros(Sensor.Num, 1);

% Errors 
err_x = zeros(1, Array.N);
err_y = zeros(1, Array.N);

%% Simulation

for k = 1:Array.N

    Env.RefPosition = Array.ActualPos(k,:);

    Sensor.PosMeas = zeros(Sensor.Num, 2);
    Sensor.Detect = zeros(Sensor.Num, 1);
    DistSensPos = zeros(Sensor.Num, 1);

    for i = 1:Sensor.Num
          DistSensPos(i) = norm(Sensor.Position(i,:) - Env.RefPosition);
          % Distance Check and Measurement
          if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) && (Sensor.Detect(i) == 0) 
                Sensor.PosMeas(i,:) = Env.RefPosition + mvnrnd(Sensor.PosMu, Sensor.PosCov);
                Sensor.Detect(i) = 1;

                while Sensor.PosMeas(i,1) <= 0 || Sensor.PosMeas(i,1) >= Room.Width || ...
                        Sensor.PosMeas(i,2) <= 0 || Sensor.PosMeas(i,2) >= Room.Height

                        Sensor.PosMeas(i,:) = Env.RefPosition +  mvnrnd(Sensor.PosMu, Sensor.PosCov);
                end 
          end
    end

    [MinDist, MinIndex] = min(DistSensPos);

    EstimatedPos(k,1) = Sensor.PosMeas(MinIndex,1); 
    EstimatedPos(k,2) = Sensor.PosMeas(MinIndex,2);

    UsedPos(k,:) = Env.RefPosition;
  
    err_x(k) = EstimatedPos(k,1) - Env.RefPosition(1);
    err_y(k) = EstimatedPos(k,2) - Env.RefPosition(2);
end


%% Errors analysis


mean_err_x = mean(err_x);
mean_err_y = mean(err_y);

std_err_x = std(err_x);
std_err_y = std(err_y);

max_err_x = max(err_x);
max_err_y = max(err_y);

rmse_x = rmse(Array.ActualPos(:,1), EstimatedPos(:,1));
rmse_y = rmse(Array.ActualPos(:,1), EstimatedPos(:,2));

disp(' ')
disp('Errors in Position Estimation without Consensus Algorithm')
disp(['The mean error on x coordinate is: ', num2str(mean_err_x, 3), ' m']);
disp(['The mean error on y coordinate is: ', num2str(mean_err_y, 3), ' m']);

disp(['The standard deviation on x coordinate is: ', num2str(std_err_x, 3), ' m']);
disp(['The standard deviation on y coordinate is: ', num2str(std_err_y, 3), ' m']);
 
disp(['The maximum error on x coordinate is: ', num2str(max_err_x, 3), ' m']);
disp(['The maximum error on y coordinate is: ', num2str(max_err_y, 3), ' m']);

disp(['The root mean squared error on x coordinate is: ', num2str(rmse_x, 3), ' m']);
disp(['The root mean squared error on y coordinate is: ', num2str(rmse_y, 3), ' m']);

%{
% Plot errors in x-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Error of Position Estimation in X-coordinate','FontSize',14)
hold on;
plot(1:Array.N,err_x);
% xticks(1:Array.N);
hold off
 
% Plot errors in y-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Error of Position Estimation in Y-coordinate','FontSize',14)
hold on;
plot(1:Array.N,err_y);
% xticks(1:Array.N);
hold off
%}

%{
% Histograms
figure
histogram(err_x);

figure
histogram(err_y);
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
% for k=1:length(SetUp.Time)-1
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
