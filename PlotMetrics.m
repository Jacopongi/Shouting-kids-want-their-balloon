function PlotMetrics(KidArray)
% This function is used to plot the KPI's of the different cases to
% verify the validity of the social experiment.
% KPI's:
%   - Time until arrival at destination
%   - Total distance of traveled path
%   - others?

% how to best compare them? What types of statistical analysis?
% write them in a way that returns the plots of each category for all cases
%   => at the end we could run every case let's say 50 times and make
%   histograms, bell curves, medians, etc... ??
%% Prepare data
x = 1:KidArray.N;
y1 = KidArray.TravelTime;
y2 = KidArray.PathLength;

figure(6), clf

% Grouped bar chart with two y-axes for time and path length
xlim([0.1, KidArray.N+0.9])
yyaxis left; 
b1 = bar(x - 0.15, y1, 0.3);    % 0.3 means 30%Plo of total space available used
                                % -0.15 means shifted to the left so
                                % they're next to each other
b1.FaceColor = "#0072BD";                                            
ylabel('Total travel time [s]', "Color", "#0072BD");

yyaxis right; 
b2 = bar(x + 0.15, y2, 0.3);
b2.FaceColor = "#A2142F";
ylabel('Total path length [m]', "Color", "#A2142F");  
% !!! this needs to be updated still, currently the values are actually in cm

% Customize the plot
xlabel('Kids');
title('Comparison of results');
% legend('Travel time', 'Path length');
legend({'Travel time', 'Path length'}, 'Location', 'best')
grid on;

% Adjust the second y-axis
ax = gca;  
ax.YAxis(2).Color = "#A2142F";  



%% Compare the average velocity with desired one
% Average velocity computed by total path length / travel time
AvgVel = KidArray.PathLength ./ KidArray.TravelTime;
    % problem of units still remains

y3 = KidArray.DesiredVel;

figure(7), clf
b3 = bar(x, AvgVel, 0.3);
b3.FaceColor = "#7E2F8E";
hold on
% make a point for the desiredvelocity of each kid
plot(x, y3, 'LineStyle', "none", 'LineWidth', 1, ...
            'Marker', "diamond", 'Color', "#4DBEEE")

xlabel('Kids')
ylabel('Velocity [m/s]')
title('Velocities - actual vs desired')
legend({'Actual vel (average)', 'Desired vel (max)'}, 'Location', 'best')


%% Compare the direct/straight path distance (SPL) with the actual TPL
% SPL computed by eucl norm of destination to initial position
y4 = diag(pdist2(KidArray.Destinations, KidArray.InitPos));
    % problem of units still remains
% y2 = TPL

figure(8), clf
b4 = bar(x, y4, 0.3);
b4.FaceColor = "#77AC30";
hold on
plot(x, y2, 'LineStyle', "none", 'LineWidth', 1, ...
            'Marker', "diamond", 'Color', "#D95319")

xlabel('Kids')
ylabel('Distance [m]')
title('Path length - direct vs actual')
legend({'Direct/straight', 'Actual'}, 'Location', 'best')

% !!! units m and cm


%% Intermediate conclusion
% Write this into report:
% we observe that the kids with the highest discrepancies between their
% total travel time (TTT) and their total path length (TPL) (fig.6) not
% necessarily are the fastest ones (fig.7)



end