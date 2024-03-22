function [KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, RoomWidth, RoomHeight)

KidArray.N = numKid;
BalloonArray.N = numBal;

% Dimensions 
KidArray.Radius = 70;   % [cm]  0.7;          % [m]
BalloonArray.Edge = 35; % [cm]  0.35;       % [m]
MinimumDistance = KidArray.Radius + BalloonArray.Edge;

% Field to measure the traveled distance and time it took
KidArray.PathLength = zeros(numKid, 1);
KidArray.TravelTime = zeros(numKid, 1);
KidArray.InitPos = zeros(numKid, 1);    % for the plots

% ID
KidArray.ID = (1:numKid)';
BalloonArray.ID = (1:numBal)';

% Bounds on velocity
maxVel = 220;   % [cm/s]    2.2;   % [m/s]
minVel = 50;    % [cm/s]    0.5;   % [m/s]


% Random starting Positions 
KidArray.Positions = rand(numKid,2);
KidArray.InitPos = KidArray.Positions;
KidArray.Destinations = zeros(numKid,2);
BalloonArray.Positions = rand(numBal,2);

ScaleFactor = 0.1; % Note! Between 0 and 1!
                   % Needed to avoid generation on the walls

KidArray.Positions(:,1) = ScaleFactor * RoomWidth + KidArray.Positions(:,1) * (1- 2*ScaleFactor) * RoomWidth;
KidArray.Positions(:,2) = ScaleFactor * RoomHeight + KidArray.Positions(:,2) * (1-2*ScaleFactor) * RoomHeight;

BalloonArray.Positions(:,1) = ScaleFactor * RoomWidth + BalloonArray.Positions(:,1) * (1-2*ScaleFactor) * RoomWidth;
BalloonArray.Positions(:,2) = ScaleFactor * RoomHeight + BalloonArray.Positions(:,2) * (1-2*ScaleFactor) * RoomHeight;

% Check distance between kids
OkayDistance = 0;

while ( OkayDistance == 0 )
    dist_KidKid = 1;
    dist_BalBal = 1;
    dist_KidBal = 1;

    % Check distance between kids
    for i = 1:numKid
            for j = 1:numKid
                if i==j
                    continue;
                end 
                if ( sqrt( (KidArray.Positions(i,1) - KidArray.Positions(j,1)).^2 + (KidArray.Positions(i,2) - KidArray.Positions(j,2)).^2) < 2*MinimumDistance) 
                    dist_KidKid = 0;
                    break;
                end
            end
    end

    % Check distance between balloons
    for i = 1:numBal
            for j = 1:numBal
                if i==j
                    continue;
                end 
                if ( sqrt( (BalloonArray.Positions(i,1) - BalloonArray.Positions(j,1)).^2 + (BalloonArray.Positions(i,2) - BalloonArray.Positions(j,2)).^2) < 2*MinimumDistance) 
                    dist_BalBal = 0;
                    break;
                end
            end
    end

    % Check distance bewteen kids and balloons 
    for i = 1:numKid
            for j = 1:numBal
                if ( sqrt( (KidArray.Positions(i,1) - BalloonArray.Positions(j,1)).^2 + (KidArray.Positions(i,2) - BalloonArray.Positions(j,2)).^2) < 2*MinimumDistance) 
                    dist_KidBal = 0;
                    break;
                end
            end     
    end

    if (dist_KidKid == 0) || (dist_BalBal == 0) || (dist_KidBal == 0)

        % Regenerate positions
        KidArray.Positions = rand(numKid,2);
        BalloonArray.Positions = rand(numBal,2);
        
        KidArray.Positions(:,1) = ScaleFactor * RoomWidth + KidArray.Positions(:,1) * (1-2*ScaleFactor) * RoomWidth;
        KidArray.Positions(:,2) = ScaleFactor * RoomHeight + KidArray.Positions(:,2) * (1-2*ScaleFactor) * RoomHeight;
        
        BalloonArray.Positions(:,1) = ScaleFactor * RoomWidth + BalloonArray.Positions(:,1) * (1-2*ScaleFactor) * RoomWidth;
        BalloonArray.Positions(:,2) = ScaleFactor * RoomHeight + BalloonArray.Positions(:,2) * (1-2*ScaleFactor) * RoomHeight;
    else 
        OkayDistance = 1;
    end
end


% Random x and y velocities
KidArray.DesiredVel = rand(numKid,1)*(maxVel-minVel) + minVel;
KidArray.ActualVel = zeros(numKid,2);

% Figure 
figure
axis equal
axis([0 RoomWidth 0 RoomHeight])
box on
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
title('Kids in the room','FontSize',14)

circlefig = zeros(1,numKid);
KidArray.Color = rand(numKid,3);
for i = 1:numKid
    x_min = KidArray.Positions(i,1) - KidArray.Radius;
    y_min = KidArray.Positions(i,2) - KidArray.Radius;
    radius_cur = KidArray.Radius;
    circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
        'Curvature',[1 1], 'FaceColor',KidArray.Color(i,:));
    text(KidArray.Positions(i,1), KidArray.Positions(i,2), num2str(KidArray.ID(i)), 'HorizontalAlignment', 'center', 'Color','k');
end

squarefig = zeros(1,numBal);

for i = 1:numBal
    x_min_b = BalloonArray.Positions(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.Positions(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], 'FaceColor',KidArray.Color(i,:));
    text(BalloonArray.Positions(i,1), BalloonArray.Positions(i,2), num2str(BalloonArray.ID(i)), 'HorizontalAlignment', 'center', 'Color','k');
end


end