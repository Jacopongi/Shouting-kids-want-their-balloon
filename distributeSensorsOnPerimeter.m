function SensorsPosition = distributeSensorsOnPerimeter(numSensors, rectWidth, rectHeight)
    % numSensors: Number of sensors to distribute
    % rectWidth: Width of the rectangle
    % rectHeight: Height of the rectangle

    % Divide the angular space in equal slices
    a = linspace(0, 2*pi, numSensors+1);
    r = rectWidth + rectHeight;
    xx = r*cos(a) + rectWidth/2;
    yy = r*sin(a) + rectHeight/2;

    % Initialize figure
    figure;
    axis equal;
    axis([-0.2*rectWidth 1.2*rectWidth -0.2*rectHeight 1.2*rectHeight])
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Sensors on the wall','FontSize',14)

    hold on;

    % Build the room
    xlimit = [0 rectWidth];
    ylimit = [0  rectHeight];
    xbox = xlimit([1 1 2 2 1]);
    ybox = ylimit([1 2 2 1 1]);
    rectangle('Position', [0, 0, rectWidth, rectHeight], 'LineWidth', 2);

    % Build the sensors
    SensorsPosition = zeros(numSensors,2);
    for i = 1:numSensors
        x_aux = [rectWidth/2 xx(i)];
        y_aux = [rectHeight/2 yy(i)];
        [xi, yi] = intersections(x_aux, y_aux, xbox, ybox,1);
        SensorsPosition(i,:) = [xi, yi]; 

        plot(xi, yi, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end

    hold off;

end
