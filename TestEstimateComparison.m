function [] = TestEstimateComparison(Test)
% Summary: Comparison btw estimations with and without Consensus Algorithm
% Description: Errors obtained from TestEstimatePosition and
% TestEstimatePositionNoCons functions are imported.
% Mean, standard deviation, maximum error and root mean squared error
% are computed for both cases and compared.

err_x = Test.err_x;
err_y = Test.err_y;
err_x_NC = Test.err_x_NC;
err_y_NC = Test.err_y_NC;

EstimatedPos = Test.EstimatedPos;
EstimatedPosNC = Test.EstimatedPosNC;
RefPositions = Test.ReferencePos;

binwidth = 0.005;

%% Errors with Consensus Algorithm

mean_err_x = mean(err_x);
mean_err_y = mean(err_y);

std_err_x = std(err_x);
std_err_y = std(err_y);

max_err_x = max(abs(err_x));
max_err_y = max(abs(err_y));

rmse_x = rmse(RefPositions(:,1), EstimatedPos(:,1));
rmse_y = rmse(RefPositions(:,2), EstimatedPos(:,2));

disp(' ')
disp('Errors in Position Estimation with Consensus algorithm')

disp(['The mean error on x coordinate is: ', num2str(mean_err_x, 3), ' m']);
disp(['The mean error on y coordinate is: ', num2str(mean_err_y, 3), ' m']);

disp(['The standard deviation on x coordinate is: ', num2str(std_err_x, 3), ' m']);
disp(['The standard deviation on y coordinate is: ', num2str(std_err_y, 3), ' m']);
 
disp(['The maximum error on x coordinate is: ', num2str(max_err_x, 3), ' m']);
disp(['The maximum error on y coordinate is: ', num2str(max_err_y, 3), ' m']);

disp(['The root mean squared error on x coordinate is: ', num2str(rmse_x, 3), ' m']);
disp(['The root mean squared error on y coordinate is: ', num2str(rmse_y, 3), ' m']);



%{
% Plot errors on x-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Errors on X-coordinate with consensus estimation','FontSize',14)
hold on;
plot(1:length(EstimatedPos), err_x)
% xticks(1:length(EstimatedPos));
hold off
 
% Plot errors on y-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Errors on Y-coordinate with consensus estimation','FontSize',14)
hold on;
plot(1:length(EstimatedPos), err_y)
% xticks(1:length(EstimatedPos));
hold off
%}

figure
histogram(err_x)
title('Errors on X-coordinate with consensus estimation','FontSize',14)

figure
histogram(err_x,'Normalization','pdf', 'BinWidth', binwidth)
hold on
x = sort(err_x);
mu = mean(err_x);
sigma = std(err_x);
f = exp(-(x-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(x, f,'LineWidth', 1.5)
title('Errors on X-coordinate with consensus estimation','FontSize',14)
hold off

figure
histogram(err_y)
title('Errors on Y-coordinate with consensus estimation','FontSize',14)

figure
histogram(err_y,'Normalization','pdf', 'BinWidth', binwidth)
hold on
y = sort(err_y);
mu = mean(err_y);
sigma = std(err_y);
f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(y, f,'LineWidth', 1.5)
title('Errors on Y-coordinate with consensus estimation','FontSize',14)
hold off


%% Errors without Consensus Algorithm (Single estimation)

mean_err_x_NC = mean(err_x_NC);
mean_err_y_NC = mean(err_y_NC);

std_err_x_NC = std(err_x_NC);
std_err_y_NC = std(err_y_NC);

max_err_x_NC = max(abs(err_x_NC));
max_err_y_NC = max(abs(err_y_NC));

rmse_x = rmse(RefPositions(:,1), EstimatedPosNC(:,1));
rmse_y = rmse(RefPositions(:,2), EstimatedPosNC(:,2));

disp(' ')
disp('Errors in Position Estimation without Consensus algorithm')

disp(['The mean error on x coordinate is: ', num2str(mean_err_x_NC, 3), ' m']);
disp(['The mean error on y coordinate is: ', num2str(mean_err_y_NC, 3), ' m']);

disp(['The standard deviation on x coordinate is: ', num2str(std_err_x_NC, 3), ' m']);
disp(['The standard deviation on y coordinate is: ', num2str(std_err_y_NC, 3), ' m']);
 
disp(['The maximum error on x coordinate is: ', num2str(max_err_x_NC, 3), ' m']);
disp(['The maximum error on y coordinate is: ', num2str(max_err_y_NC, 3), ' m']);

disp(['The root mean squared error on x coordinate is: ', num2str(rmse_x, 3), ' m']);
disp(['The root mean squared error on y coordinate is: ', num2str(rmse_y, 3), ' m']);

disp(' ')

%{
% Plot errors on x-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Errors on X-coordinate with single estimation','FontSize',14)
hold on;
plot(1:length(EstimatedPos), err_x_NC)
% xticks(1:length(EstimatedPos));
hold off
 
% Plot errors on y-position estimation
figure;
box on
grid on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Errors on Y-coordinate with single estimation','FontSize',14)
hold on;
plot(1:length(EstimatedPos), err_y_NC)
% xticks(1:length(EstimatedPos));
hold off
%}

figure
histogram(err_x_NC);
title('Errors on X-coordinate with single estimation','FontSize',14)

figure
histogram(err_x_NC,'Normalization','pdf', 'BinWidth', binwidth)
hold on
x = sort(err_x_NC);
mu = mean(err_x_NC);
sigma = std(err_x_NC);
f = exp(-(x-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(x, f,'LineWidth', 1.5)
title('Errors on X-coordinate with single estimation','FontSize',14)
hold off

figure
histogram(err_y_NC);
title('Errors on Y-coordinate with single estimation','FontSize',14)

figure
histogram(err_y_NC,'Normalization','pdf','BinWidth', binwidth)
hold on
y = sort(err_y_NC);
mu = mean(err_y_NC);
sigma = std(err_y_NC);
f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(y, f,'LineWidth', 1.5)
title('Errors on Y-coordinate with single estimation','FontSize',14)
hold off


%% Errors comparison

%{
figure
histogram(err_x_NC)
hold on
histogram(err_x)
hold off

figure
histogram(err_y_NC)
hold on
histogram(err_y)
hold off
%}

figure
histogram(err_x_NC,'Normalization','probability', 'BinWidth', binwidth)
title('Errors comparison on X-coordinate','FontSize',14)
hold on
histogram(err_x,'Normalization','probability', 'BinWidth', binwidth)
legend('Single Estimation', 'Consensus Estimation')
hold off


figure
histogram(err_y_NC,'Normalization','probability', 'BinWidth', binwidth)
title('Errors comparison on Y-coordinate','FontSize',14)
hold on
histogram(err_y,'Normalization','probability', 'BinWidth', binwidth)
legend('Single Estimation', 'Consensus Estimation')
hold off

% NOTE: With this normalization, the height of each bar is equal 
% to the probability of selecting an observation withon that bon interval,
% and the height of all of the bars sums to 1.



end