function SensorsPosition = distributeSensorsOnPerimeter(numSensors, rectWidth, rectHeight, PlotTag)
% Summary: given a set of sensors, they are uniformly distributed on the walls of the room.
% Description: according to the number of sensors to be positioned and room dimensions,
% angular space is divided in equal slices.
% Sensors are positioned in the intersections between cut lines and walls. 
% A plot is available for visualization.

% numSensors: Number of sensors to distribute
% rectWidth: Width of the rectangle
% rectHeight: Height of the rectangle

% Divide the angular space in equal slices
a = linspace(0, 2*pi, numSensors+1);
r = rectWidth + rectHeight;
xx = r*cos(a) + rectWidth/2;
yy = r*sin(a) + rectHeight/2;

% Build the room
xlimit = [0 rectWidth];
ylimit = [0  rectHeight];
xbox = xlimit([1 1 2 2 1]);
ybox = ylimit([1 2 2 1 1]);

% Locate the sensors
SensorsPosition = zeros(numSensors,2);
for i = 1:numSensors
    x_aux = [rectWidth/2 xx(i)];
    y_aux = [rectHeight/2 yy(i)];
    [xi, yi] = intersections(x_aux, y_aux, xbox, ybox,1);
    SensorsPosition(i,:) = [xi, yi]; 
end

% Room with sensors plot
if PlotTag  
    figure;
    axis equal;
    axis([-0.2*rectWidth 1.2*rectWidth -0.2*rectHeight 1.2*rectHeight])
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Sensors on the wall','FontSize',14)
    hold on;

    % Room
    rectangle('Position', [0, 0, rectWidth, rectHeight], 'LineWidth', 2);

    % Sensors
    for i = 1:numSensors
        plot(SensorsPosition(i,1), SensorsPosition(i,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end
    hold off;
end

end
