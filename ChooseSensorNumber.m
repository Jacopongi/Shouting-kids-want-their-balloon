function [Sensor] = ChooseSensorNumber(min_SensorNum,max_SensorNum, Room)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


% Sensor parameters
Sensor.Res = 1;
Sensor.Mu = 0;
Sensor.Sigma = Sensor.Res * rand(1)*10;
Sensor.Range = min(Room.Width, Room.Height);
Sensor.H = eye(2);
    
% Errors storage
Store.max_err_x = zeros(1, max_SensorNum - min_SensorNum);
Store.max_err_y = zeros(1, max_SensorNum - min_SensorNum);
Store.mean_err_x = zeros(1, max_SensorNum - min_SensorNum);
Store.mean_err_y = zeros(1, max_SensorNum - min_SensorNum);

for sn = min_SensorNum:max_SensorNum

    Sensor.Num = sn;
    Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, false);

    % Position Measurements
    Env.RefPosition = zeros(1,2);
    Sensor.PosMu = zeros(2,1);
    Sensor.PosCov = rand(2,2)*0.5;
    Sensor.PosCov = Sensor.PosCov'*Sensor.PosCov;
    Sensor.PosMeas = zeros(Sensor.Num, 2);
    Sensor.Detect = zeros(Sensor.Num,1);

    % Reference Positions
    gridnum = 4;
    xPos = linspace(0.1*Room.Width, 0.9*Room.Width,gridnum);
    yPos = linspace(0.1*Room.Height, 0.9*Room.Height,gridnum);
    Env.Positions = table2array(combinations(xPos, yPos));
    
    % Consensus structures
    Sensor.Cons.Adj = zeros(Sensor.Num, Sensor.Num); 
    Sensor.Cons.MsgNum = 10;
    Sensor.Cons.Degree = zeros(1, Sensor.Num);
    Sensor.Cons.F = cell(1, Sensor.Num);
    Sensor.Cons.a = cell(1, Sensor.Num);
   
    % Set-up and storage
    SetUp.Dt = 1;                         % [s]
    SetUp.Time = 0:SetUp.Dt:gridnum^2;    % [s]
    UsedPos = zeros(length(SetUp.Time)-1, 2);

    % Errors 
    err_x = zeros(1, length(SetUp.Time)-1);
    err_y = zeros(1, length(SetUp.Time)-1);
    
    % Simulation
    for k=1:length(SetUp.Time)-1
    
        Env.RefPosition = Env.Positions(k,:);
        Sensor.Detect = zeros(Sensor.Num,1);
    
        for i = 1:Sensor.Num
              % Distance Check and Measurement
              if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) && (Sensor.Detect(i) == 0) 
                    Sensor.PosMeas(i,:) = Env.RefPosition + mvnrnd(Sensor.PosMu, Sensor.PosCov);
                    Sensor.Detect(i) = 1;
              end
        end
    
        % Network discovering
        for i = 1:Sensor.Num              
            for j = 1:Sensor.Num
                % Check the distance between sensors
                % They can talk only if they are in range
                if (sqrt(sum((Sensor.Position(i,:) - Sensor.Position(j,:)).^2)) <= Sensor.Range - abs(randn(1)*Sensor.Sigma)) ...
                        && not(j==i)
                    Sensor.Cons.Adj(j,i) = 1;
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
      
        err_x(k) = mean(nonzeros(StoreEst(2:end,:,1)),"all") - Env.RefPosition(1);
        err_y(k) = mean(nonzeros(StoreEst(2:end,:,2)),"all") - Env.RefPosition(2);
    
        UsedPos(k,:) = Env.RefPosition;
        
        % Plot the estimates at each time step as a function of the number of messages exchanged by the consensus protocol
        %{
        if sum(Sensor.Detect) >= 2
            figure, clf, hold on;
            LegS = {};
            for i=1:Sensor.Num
                plot(0:Sensor.Cons.MsgNum, StoreEst(i,:,1));
                LegS{end+1} = ['r_{', num2str(i), '}'];
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
                LegS{end+1} = ['r_{', num2str(i), '}']; 
            end
            plot(0:Sensor.Cons.MsgNum, Env.RefPosition(2)*ones(1,Sensor.Cons.MsgNum+1), 'r--');
            LegS{end+1} = 'Actual Position';
            xlabel('Number of Messages');
            ylabel('y [m]');
            legend(LegS, 'Location', 'best');
            hold off
        end
        %}
    
        % Sensors with used position at each time step plot 
        %{
        figure;
        axis equal;
        axis([-0.2*Room.Width 1.2*Room.Width -0.2*Room.Height 1.2*Room.Height])
        box on
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        title('Sensors Position Estimation','FontSize',14)
        hold on;
        
        rectangle('Position', [0, 0, Room.Width, Room.Height], 'LineWidth', 2);
        plot(Env.RefPosition(1), Env.RefPosition(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        
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
   
    % Errors plot
    %{
    % Plot errors in x-position estimation
    figure;
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Error in X-Position Estimation','FontSize',14)
    hold on;
    plot(0:length(SetUp.Time)-2,err_x);
    hold off
    
    % Plot errors in y-position estimation
    figure;
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Errors in Y-Position Estimation','FontSize',14)
    hold on;
    plot(0:length(SetUp.Time)-2,err_y);
    hold off
    %}
    
    % Plot sensors and all used positions 
    %{
    figure;
    axis equal;
    axis([-0.2*Room.Width 1.2*Room.Width -0.2*Room.Height 1.2*Room.Height])
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title(['Position Estimation with ' num2str(Sensor.Num), ' Sensors'],'FontSize',14)
    hold on;
    
    rectangle('Position', [0, 0, Room.Width, Room.Height], 'LineWidth', 2);
    for i = 1:length(SetUp.Time)-1
        plot(UsedPos(i,1), UsedPos(i,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        text(UsedPos(i,1), UsedPos(i,2), num2str(i), 'HorizontalAlignment', 'center');
        if (i<length(SetUp.Time)-1)
            p1 = [UsedPos(i,1), UsedPos(i,2)];
            p2 = [UsedPos(i+1,1), UsedPos(i+1,2)];
            dp = p2 - p1;
            quiver(p1(1),p1(2),dp(1),dp(2),0.90, 'Color','b');
        end
    end
    
    for i = 1:Sensor.Num 
            plot(Sensor.Position(i,1), Sensor.Position(i,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            text(Sensor.Position(i,1), Sensor.Position(i,2), num2str(i), 'HorizontalAlignment', 'center');
    end
    hold off
    %}

    % Store the information of the sensors
    Store.Sensors(Sensor.Num) = Sensor;

    % Compute and store maximum and mean errors
    max_err_x = max(abs(err_x));
    Store.max_err_x(Sensor.Num - min_SensorNum +1) = max_err_x;
    max_err_y = max(abs(err_y));
    Store.max_err_y(Sensor.Num - min_SensorNum +1) = max_err_y;
    mean_err_x = mean(abs(err_x));
    Store.mean_err_x(Sensor.Num - min_SensorNum +1) = mean_err_x;
    mean_err_y = mean(abs(err_y));
    Store.mean_err_y(Sensor.Num - min_SensorNum +1) = mean_err_y;
    
    % Display informations about errors
    %{
    disp( ['Sensor number: ', num2str(Sensor.Num)]);
    disp( ['Maximum error on X: ', num2str(max_err_x)] );
    disp( ['Maximum error on Y: ', num2str(max_err_y)] );    
    disp( ['Mean error on X: ', num2str(mean_err_x)] );    
    disp( ['Mean error on Y: ', num2str(mean_err_y)] );
    disp(' ');
    %}
        
end

%% Max and mean errors plots

%{
% Plot max errors in x-position estimation
figure;
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Max Errors in X-Position Estimation','FontSize',14)
hold on;
plot(min_SensorNum:max_SensorNum, Store.max_err_x);
hold off

% Plot max errors in y-position estimation
figure;
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Max Errors in Y-Position Estimation','FontSize',14)
hold on;
plot(min_SensorNum:max_SensorNum, Store.max_err_y);
hold off

% % Plot mean errors in x-position estimation
figure;
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Mean Error in X-Position Estimation','FontSize',14)
hold on;
plot(min_SensorNum:max_SensorNum, Store.mean_err_x);
hold off

% Plot mean errors in x-position estimation
figure;
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Mean Errors in Y-Position Estimation','FontSize',14)
hold on;
plot(min_SensorNum:max_SensorNum, Store.mean_err_y);
hold off
%}

%% Looking for the most performant number of sensors

Store.minmax_err_x = min(Store.max_err_x);
Store.minmax_err_y = min(Store.max_err_y);

minmax_err_x_index = find(Store.max_err_x == Store.minmax_err_x);
minmax_err_y_index = find(Store.max_err_y == Store.minmax_err_y);
if minmax_err_x_index == minmax_err_y_index
    Sensor.Num = minmax_err_x_index + min_SensorNum - 1;
else
    diff_from_minmax_x = Store.max_err_x(minmax_err_y_index) - Store.max_err_x(minmax_err_x_index);
    diff_from_minmax_y = Store.max_err_y(minmax_err_x_index) - Store.max_err_y(minmax_err_y_index);
    if diff_from_minmax_x < diff_from_minmax_y
        Sensor.Num = minmax_err_y_index + min_SensorNum - 1;
    elseif diff_from_minmax_x > diff_from_minmax_y
        Sensor.Num = minmax_err_x_index + min_SensorNum - 1;
    elseif diff_from_minmax_x == diff_from_minmax_y
        minmax_err_index = min(minmax_err_x_index, minmax_err_y_index);
        Sensor.Num = minmax_err_index + min_SensorNum - 1;
    end
end

%% Saving the chosen sensors and plot their deployment  

Sensor = Store.Sensors(Sensor.Num);

Sensor.MaxErr_x = Store.max_err_x(Sensor.Num - min_SensorNum + 1);
Sensor.MaxErr_y = Store.max_err_y(Sensor.Num - min_SensorNum + 1);

disp(['The chosen number of sensor is: ', num2str(Sensor.Num)]);
disp(['The maximum error on the x position is: ', num2str(Sensor.MaxErr_x)]);
disp(['The maximum error on the y position is: ', num2str(Sensor.MaxErr_y)]);

% Cleaning fields...
Sensor.Detect = zeros(Sensor.Num,1);
Sensor.Cons.MsgNum = 0;
Sensor.PosCov = zeros(2,2);

% Plot the sensors inside the room
distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, true);

% Sensors with used positions plot 
figure;
axis equal;
axis([-0.2*Room.Width 1.2*Room.Width -0.2*Room.Height 1.2*Room.Height])
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Sensors and Used Positions','FontSize',14)
hold on;

rectangle('Position', [0, 0, Room.Width, Room.Height], 'LineWidth', 2);

for k=1:length(SetUp.Time)-1

    Env.RefPosition = Env.Positions(k,:);
    plot(Env.RefPosition(1), Env.RefPosition(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        
    for i = 1:Sensor.Num 
            plot(Sensor.Position(i,1), Sensor.Position(i,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            text(Sensor.Position(i,1), Sensor.Position(i,2), num2str(i), 'HorizontalAlignment', 'center');
            if (sqrt(sum((Sensor.Position(i,:) - Env.RefPosition).^2)) <= Sensor.Range)
                % rectangle('Position', [Sensor.Position(i,1) - Sensor.Range, Sensor.Position(i,2) - Sensor.Range, ...
                %                        Sensor.Range * 2, Sensor.Range * 2], ...
                %          'Curvature',[1, 1], 'LineStyle','--');
                plot([Sensor.Position(i,1), Env.RefPosition(1)], [Sensor.Position(i,2), Env.RefPosition(2)], 'LineStyle','-.');
            end
    end
end
hold off
end