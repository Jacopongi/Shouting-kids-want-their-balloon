function SensorsPosition = distributeSensorsOnPerimeter(numSensors, rect)
    % numSensors: Number of sensors to distribute
    % rect: Struct with
    % Width: Width of the rectangle
    % Height: Height of the rectangle

    % Divide the angular space in equal slices
    a = linspace(0, 2*pi, numSensors+1);
    r = rect.Width + rect.Height;
    xx = r*cos(a) + rect.Width/2;
    yy = r*sin(a) + rect.Height/2;

    % Initialize figure
    figure(2)
    axis equal;
    axis([-0.2*rect.Width 1.2*rect.Width -0.2*rect.Height 1.2*rect.Height])
    box on
    xlabel('x','FontSize',16)
    ylabel('y','FontSize',16)
    title('Sensors on the wall','FontSize',14)

    hold on;

    % Build the room
    xlimit = [0 rect.Width];
    ylimit = [0  rect.Height];
    xbox = xlimit([1 1 2 2 1]);
    ybox = ylimit([1 2 2 1 1]);
    rectangle('Position', [0, 0, rect.Width, rect.Height], 'LineWidth', 2);

    % Build the sensors
    SensorsPosition = zeros(numSensors,2);
    for i = 1:numSensors
        x_aux = [rect.Width/2 xx(i)];
        y_aux = [rect.Height/2 yy(i)];
        [xi, yi] = intersections(x_aux, y_aux, xbox, ybox,1);
        SensorsPosition(i,:) = [xi, yi]; 

        plot(xi, yi, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    end

    hold off;

end
