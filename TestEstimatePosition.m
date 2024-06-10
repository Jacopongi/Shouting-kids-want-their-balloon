function [Test] = TestEstimatePosition(Sensor, Room, gridnum)
% Summary: estimate several reference positions for testing purpose.
% Description: Reference positions are obtained with a grid.
% Position estimate is performed with a consensus algorithm
% exploiting Metropolis-Hastings weighting. 
% Uncertainties on estimation come from measuring system (sensors) and
% enviroment (added random noise). 
% Error between estimates and actual positions are computed to evaluate performance.

% Several plots are available but left commented for visualization ease.

% NOTE: 'gridnum' represents one dimension of the grid. Grid is symmetric.


% Positions instantiation
EstimatedPos = zeros(gridnum^2, 2);
Env.RefPosition = zeros(1,2);
UsedPos = zeros(gridnum^2, 2);

% Grid
xPos = linspace(0.1*Room.Width, 0.9*Room.Width,gridnum);
yPos = linspace(0.1*Room.Height, 0.9*Room.Height,gridnum);
Env.Positions = table2array(combinations(xPos, yPos));

% Adding External noise (independent sources affecting each dimension)
Env.NoiseCov = 0.5 * randn(2);         % Env.NoiseCov = diag(rand(2,1));           
% Alternative: Env.NoiseCov =  random('Poisson', 1, 2)*0.1;
Env.NoiseCov = Env.NoiseCov'*Env.NoiseCov;       
Sensor.PosCov = Sensor.PosCov + Env.NoiseCov;    % Remember: Covariance_Total = Covariance_Sensor + Covariance_Noise

% Sensors fields
Sensor.PosMeas = zeros(Sensor.Num, 2);
Sensor.Detect = zeros(Sensor.Num, 1);

% Consensus structures
Sensor.Cons.F = cell(1, Sensor.Num);
Sensor.Cons.a = cell(1, Sensor.Num);
Sensor.Cons.MsgNum = 10;

% Errors 
err_x = zeros(1, gridnum^2);
err_y = zeros(1, gridnum^2);

%% Simulation

for k=1:gridnum^2

    Env.RefPosition = Env.Positions(k,:);
    Test.error = 0.1 * mvnrnd(Sensor.PosMu, Sensor.PosCov, Sensor.Num);

    Sensor.Detect = zeros(Sensor.Num,1);
    Sensor.Cons.Adj = zeros(Sensor.Num, Sensor.Num);
    Sensor.Cons.Degree = zeros(1, Sensor.Num);

    for i = 1:Sensor.Num
          % Distance Check and Measurement
          if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) && (Sensor.Detect(i) == 0) 
                Sensor.PosMeas(i,:) = Env.RefPosition + Test.error(i,:);
                Sensor.Detect(i) = 1;
          end
    end

    for i = 1:Sensor.Num 
        if Sensor.Detect(i)
            for j = 1:Sensor.Num
                % Check the distance between sensors        
                % NOTE: They can talk only if they are in range!

                if (sqrt(sum((Sensor.Position(i,:) - Sensor.Position(j,:)).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) ...
                        && not(j==i) && Sensor.Detect(j)
                    Sensor.Cons.Adj(j,i) = 1;
                end
            end
        end
    end
    
    % Consensus Degree 
    for i = 1:Sensor.Num
        Sensor.Cons.Degree(i) = sum(Sensor.Cons.Adj(i,:));
    end
    
    % Composite matrices and vectors
    for i=1:Sensor.Num
         % Consensus information to be shared
         if Sensor.Detect(i)
            % Local estimate
            Sensor.Cons.F{i} = Sensor.H'*inv(Sensor.PosCov)*Sensor.H;
            Sensor.Cons.a{i} = Sensor.H'*inv(Sensor.PosCov)*Sensor.PosMeas(i,:)';
        end
    end
    
    % Consensus rounds
    StoreEst = zeros(Sensor.Num, Sensor.Cons.MsgNum+1, 2);

    for i = 1:Sensor.Num
        % Store estimate in the first column 
        if Sensor.Detect(i)
            StoreEst(i,1,:) = inv(Sensor.Cons.F{i})*Sensor.Cons.a{i};
        end
    end
        
    % Consensus algorithm
    for nMsg = 1:Sensor.Cons.MsgNum
            F = Sensor.Cons.F;
            a = Sensor.Cons.a;
        
            % Average Consensus
            for i=1:Sensor.Num
                for j=1:Sensor.Num
                    if Sensor.Cons.Adj(i,j)
                        % Metropolis-Hastings weighting
                        q_ij = 1/(max(Sensor.Cons.Degree(i), Sensor.Cons.Degree(j)) + 1);   
                        Sensor.Cons.F{i} = Sensor.Cons.F{i} + q_ij*(F{j} - F{i});
                        Sensor.Cons.a{i} = Sensor.Cons.a{i} + q_ij*(a{j} - a{i});
                    end
                end
            end

            for i=1:Sensor.Num
                if Sensor.Detect(i)
                    StoreEst(i,nMsg+1,:) = inv(Sensor.Cons.F{i})*Sensor.Cons.a{i};
                end
            end
    end

    EstimatedPos(k,1) = mean(nonzeros(StoreEst(2:end,:,1)),"all");
    EstimatedPos(k,2) = mean(nonzeros(StoreEst(2:end,:,2)),"all");

    UsedPos(k,:) = Env.RefPosition;
    Test.collect(:,:, k) = Test.error;
  
    err_x(k) = EstimatedPos(k,1) - Env.RefPosition(1);
    err_y(k) = EstimatedPos(k,2) - Env.RefPosition(2);


    % Plot the estimates at each time step as a function of the number of messages exchanged by the consensus protocol
    %{
    if sum(Sensor.Detect) >= 2
        figure, clf, hold on;
        LegS = {};
        for i=1:Sensor.Num
            plot(0:Sensor.Cons.MsgNum, StoreEst(i,:,1));
            LegS{end+1} = ['sensor_{', num2str(i), '}'];
        end
        plot(0:Sensor.Cons.MsgNum, Env.RefPosition(1)*ones(1,Sensor.Cons.MsgNum+1), 'r--');
        LegS{end+1} = 'Actual Position';
        xlabel('Number of Messages');
        ylabel('x [m]');
        legend(LegS, 'Location', 'best');
        hold off
    
        figure, clf, hold on;
        LegS = {};
        for i=1:Sensor.Num
            plot(0:Sensor.Cons.MsgNum, StoreEst(i,:,2));
            LegS{end+1} = ['sensor_{', num2str(i), '}']; 
        end
        plot(0:Sensor.Cons.MsgNum, Env.RefPosition(2)*ones(1,Sensor.Cons.MsgNum+1), 'r--');
        LegS{end+1} = 'Actual Position';
        xlabel('Number of Messages');
        ylabel('y [m]');
        legend(LegS, 'Location', 'best');
        hold off
    end
    %}
    
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
Test.EstimatedPos = EstimatedPos;
Test.ReferencePos = Env.Positions;
Test.err_x = err_x;
Test.err_y = err_y;

%% Errors analysis

%{
mean_err_x = mean(err_x);
mean_err_y = mean(err_y);

std_err_x = std(err_x);
std_err_y = std(err_y);

max_err_x = max(abs(err_x));
max_err_y = max(abs(err_y));

rmse_x = rmse(Env.Positions(:,1), EstimatedPos(:,1));
rmse_y = rmse(Env.Positions(:,2), EstimatedPos(:,2));

disp('-------------------------')
disp('Errors of Position Estimation with Consensus Algorithm')

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
title('Error of Position Estimation in X-coordinate','FontSize',14)
hold on;
plot(1:gridnum^2, err_x);
% xticks(1:gridnum^2);
hold off
 
% Plot errors in y-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Error of Position Estimation in Y-coordinate','FontSize',14)
hold on;
plot(1:gridnum^2, err_y);
% xticks(1:gridnum^2);
hold off

figure
histogram(err_x);

figure
histogram(err_x,'Normalization','pdf')
hold on
x = sort(err_x);
mu = mean(err_x);
sigma = std(err_x);
f = exp(-(x-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(x, f,'LineWidth', 1.5)
hold off

figure
histogram(err_y);

figure
histogram(err_y,'Normalization','pdf')
hold on
y = sort(err_y);
mu = mean(err_y);
sigma = std(err_y);
f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(y, f,'LineWidth', 1.5)
hold off
%}


%% All sensors and positions (actual and estimated ones)

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
