function [EstimatedPos] = EstimatePosition(Array, Sensor, Room)
% Summary: Estimates the position of a set of agents in a room.
% Description: Position estimate is performed with a consensus algorithm
% exploiting Metropolis-Hastings weighting. 
% Uncertainties on estimation come from measuring systems (sensor) and
% enviroment (added random noise). 
% Error between estimates and actual positions are computed to evaluate performance.


% Position Measurements
EstimatedPos = zeros(Array.N, 2);
Env.RefPosition = zeros(1,2);
Env.NoiseCov = diag(rand(2,1));                  % Adding External noise (independent sources affecting each dimension)
Sensor.PosCov = Sensor.PosCov + Env.NoiseCov;    % Covariance_Total = Covariance_Sensor + Covariance_Noise 
Sensor.PosMeas = zeros(Sensor.Num, 2);
Sensor.Detect = zeros(Sensor.Num, 1);

% Consensus structures
Sensor.Cons.F = cell(1, Sensor.Num);
Sensor.Cons.a = cell(1, Sensor.Num);

% Set-up and storage
SetUp.Dt = 1;                          % [s]
SetUp.Time = 0:SetUp.Dt:Array.N;       % [s]
UsedPos = zeros(length(SetUp.Time)-1, 2);

% Errors 
err_x = zeros(1, length(SetUp.Time)-1);
err_y = zeros(1, length(SetUp.Time)-1);

% Simulation
for k=1:length(SetUp.Time)-1

    Env.RefPosition = Array.ActualPos(k,:);
    Sensor.Detect = zeros(Sensor.Num,1);

    for i = 1:Sensor.Num
          % Distance Check and Measurement
          if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) && (Sensor.Detect(i) == 0) 
                Sensor.PosMeas(i,:) = Env.RefPosition + mvnrnd(Sensor.PosMu, Sensor.PosCov);
                Sensor.Detect(i) = 1;  
            
                while Sensor.PosMeas(i,1) <= 0 || Sensor.PosMeas(i,1) >= Room.Width || Sensor.PosMeas(i,2) <= 0 || Sensor.PosMeas(i,2) >= Room.Height
                   Sensor.PosMeas(i,:) = Env.RefPosition + mvnrnd(Sensor.PosMu, Sensor.PosCov);
                end
          end
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
    FloodComplete = 0;
    FloodTag = zeros(1, Sensor.Num);

    for i = 1:Sensor.Num
        % if we have a new measurement we store the estimate in the first column 
        if Sensor.Detect(i)
            StoreEst(i,1,:) = inv(Sensor.Cons.F{i})*Sensor.Cons.a{i};
            FloodTag(i) = 1;
        else
            Sensor.Cons.F{i} = zeros(2);
            Sensor.Cons.a{i} = zeros(2,1);
        end
    end

    % Share information with all the sensors
    while FloodComplete == 0
        for i=1:Sensor.Num
            if not(Sensor.Detect(i))
                for j=1:Sensor.Num
                    if ( Sensor.Cons.Adj(i,j) ) && ( nnz(nonzeros(Sensor.Cons.F{j})) > 0)
                        Sensor.Cons.F{i} = Sensor.Cons.F{j};
                        Sensor.Cons.a{i} = Sensor.Cons.a{j};
                        StoreEst(i,1,:) = inv(Sensor.Cons.F{i})*Sensor.Cons.a{i} + rand(1);
                        FloodTag(i) = 1;
                        break;
                    end 
                end
            end
        end

        if all(FloodTag > 0.5)
                    FloodComplete = 1;
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
                StoreEst(i,nMsg+1,:) = inv(Sensor.Cons.F{i})*Sensor.Cons.a{i};
            end
    end

    EstimatedPos(k,1) = mean(nonzeros(StoreEst(2:end,:,1)),"all");
    EstimatedPos(k,2) = mean(nonzeros(StoreEst(2:end,:,2)),"all");

    UsedPos(k,:) = Env.RefPosition;
  
    err_x(k) = mean(nonzeros(StoreEst(2:end,:,1)),"all") - Env.RefPosition(1);
    err_y(k) = mean(nonzeros(StoreEst(2:end,:,2)),"all") - Env.RefPosition(2);

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

%% Errors

mean_err_x = mean(err_x);
mean_err_y = mean(err_y);

max_err_x = max(err_x);
max_err_y = max(err_y);

% disp('-------------------------')
% disp('Errors of Position Estimation')
% disp(['The mean error on x coordinate is: ', num2str(mean_err_x, 3), ' m']);
% disp(['The mean error on y coordinate is: ', num2str(mean_err_y, 3), ' m']);
% 
% disp(['The maximum error on x coordinate is: ', num2str(max_err_x, 3), ' m']);
% disp(['The maximum error on y coordinate is: ', num2str(max_err_y, 3), ' m']);
% disp('-------------------------')


% % Plot errors in x-position estimation
% figure;
% box on
% xlabel('x','FontSize',16)
% ylabel('y','FontSize',16)
% title('Error of Position Estimation in X-coordinate','FontSize',14)
% hold on;
% plot(0:length(SetUp.Time)-2,err_x);
% hold off
% 
% % Plot errors in y-position estimation
% figure;
% box on
% xlabel('x','FontSize',16)
% ylabel('y','FontSize',16)
% title('Error of Position Estimation in Y-coordinate','FontSize',14)
% hold on;
% plot(0:length(SetUp.Time)-2,err_y);
% hold off

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
