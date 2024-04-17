function [KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, RoomWidth, RoomHeight)

KidArray.N = numKid;
BalloonArray.N = numBal;

% Dimensions 
KidArray.Radius =   0.5;   % [m] 
BalloonArray.Edge = 0.7;   % [m]

MinimumDistance = KidArray.Radius + BalloonArray.Edge;

% Additional useful fields
% - to measure the traveled distance and time --> evaluation of results
KidArray.PathLength = zeros(numKid, 1);
KidArray.TravelTime = zeros(numKid, 1);
KidArray.InitPos = zeros(numKid, 2); 
BalloonArray.InitPos = zeros(numBal, 2);
% - for the plots
KidArray.circlefig = zeros(1,numKid);
BalloonArray.squarefig = zeros(1,numBal);
% - remember ID's (row where ID is saved) of kids that have arrived
KidArray.ID_arr = (1:numKid)';
BalloonArray.ID_arr = (1:numBal)';
% - remember which balloons have been visited by each kid (only case 2)
KidArray.BalVisited = zeros(numKid);

% ID
KidArray.ID = (1:numKid)';
BalloonArray.ID = (1:numBal)';

% Bounds on velocity

maxVel = 2.2;   % [m/s]     
minVel = 0.5;   % [m/s]   

% Positions estimated by sensors 
KidArray.EstimatedPos = zeros(numKid,2); 

% Random starting Positions 
KidArray.ActualPos = rand(numKid,2);
KidArray.Destinations = zeros(numKid,2);
BalloonArray.ActualPos = rand(numBal,2);

ScaleFactor = 0.1; % Note! Between 0 and 1!
                   % Needed to avoid generation on the walls

KidArray.ActualPos(:,1) = ScaleFactor * RoomWidth + KidArray.ActualPos(:,1) * (1- 2*ScaleFactor) * RoomWidth;
KidArray.ActualPos(:,2) = ScaleFactor * RoomHeight + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * RoomHeight;

BalloonArray.ActualPos(:,1) = ScaleFactor * RoomWidth + BalloonArray.ActualPos(:,1) * (1-2*ScaleFactor) * RoomWidth;
BalloonArray.ActualPos(:,2) = ScaleFactor * RoomHeight + BalloonArray.ActualPos(:,2) * (1-2*ScaleFactor) * RoomHeight;


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
                if ( sqrt( (KidArray.ActualPos(i,1) - KidArray.ActualPos(j,1)).^2 + (KidArray.ActualPos(i,2) - KidArray.ActualPos(j,2)).^2) < 2*MinimumDistance) 
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
                if ( sqrt( (BalloonArray.ActualPos(i,1) - BalloonArray.ActualPos(j,1)).^2 + (BalloonArray.ActualPos(i,2) - BalloonArray.ActualPos(j,2)).^2) < 2*MinimumDistance) 
                    dist_BalBal = 0;
                    break;
                end
            end
    end

    % Check distance bewteen kids and balloons 
    for i = 1:numKid
            for j = 1:numBal
                if ( sqrt( (KidArray.ActualPos(i,1) - BalloonArray.ActualPos(j,1)).^2 + (KidArray.ActualPos(i,2) - BalloonArray.ActualPos(j,2)).^2) < 2*MinimumDistance) 
                    dist_KidBal = 0;
                    break;
                end
            end     
    end

    if (dist_KidKid == 0) || (dist_BalBal == 0) || (dist_KidBal == 0)

        % Regenerate positions
        KidArray.ActualPos = rand(numKid,2);
        BalloonArray.ActualPos = rand(numBal,2);
        
        KidArray.ActualPos(:,1) = ScaleFactor * RoomWidth + KidArray.ActualPos(:,1) * (1-2*ScaleFactor) * RoomWidth;
        KidArray.ActualPos(:,2) = ScaleFactor * RoomHeight + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * RoomHeight;
        
        BalloonArray.ActualPos(:,1) = ScaleFactor * RoomWidth + BalloonArray.ActualPos(:,1) * (1-2*ScaleFactor) * RoomWidth;
        BalloonArray.ActualPos(:,2) = ScaleFactor * RoomHeight + BalloonArray.ActualPos(:,2) * (1-2*ScaleFactor) * RoomHeight;
    else 
        OkayDistance = 1;
    end
end

% Save initial positions
KidArray.InitPos = KidArray.ActualPos;
BalloonArray.InitPos = BalloonArray.ActualPos;

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


KidArray.Color = rand(numKid,3);
for i = 1:numKid
    x_min = KidArray.ActualPos(i,1) - KidArray.Radius;
    y_min = KidArray.ActualPos(i,2) - KidArray.Radius;
    radius_cur = KidArray.Radius;
    KidArray.circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
        'Curvature',[1 1], 'FaceColor',KidArray.Color(i,:));
    text(KidArray.ActualPos(i,1), KidArray.ActualPos(i,2), num2str(KidArray.ID(i)), ...
        'HorizontalAlignment', 'center', 'Color','k', 'FontSize', KidArray.Radius*15);
end



for i = 1:numBal
    x_min_b = BalloonArray.ActualPos(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.ActualPos(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    BalloonArray.squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor',KidArray.Color(i,:));
    text(BalloonArray.ActualPos(i,1), BalloonArray.ActualPos(i,2), num2str(BalloonArray.ID(i)), ...
        'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalloonArray.Edge*10);

end


end
