function [KidArray, BalloonArray] = distributeKidBalloon(numKid, numBal, Room)

KidArray.N = numKid;
BalloonArray.N = numBal;
KidArray.ID = 1:numKid;
BalloonArray.ID = 1:numBal;

% Dimensions 
KidArray.Radius = 0.3;
BalloonArray.Edge = 0.4;

% Random starting Positions 
KidArray.Positions = rand(numKid,2);
KidArray.Destinations = zeros(numKid,2);
BalloonArray.Positions = rand(numBal,2);

KidArray.Positions(:,1) = 0.1*Room.Width + KidArray.Positions(:,1) * 0.8*Room.Width;
KidArray.Positions(:,2) = 0.1*Room.Height + KidArray.Positions(:,2) * 0.8*Room.Height;

BalloonArray.Positions(:,1) = 0.1*Room.Width + BalloonArray.Positions(:,1) * 0.8*Room.Width;
BalloonArray.Positions(:,2) = 0.1*Room.Height + BalloonArray.Positions(:,2) * 0.8*Room.Height;

% Check distance between kids
dist_KidKid = 0;
dist_BalBal = 0;
dist_KidBal = 0;
while( (dist_KidKid == 0) || (dist_BalBal == 0) || (dist_KidBal == 0) )
        exit_for = 0;
        for i = 1:numKid
            for j = 1:numKid
                if i==j
                    continue;
                end 
                if ( sqrt( (KidArray.Positions(i,1) - KidArray.Positions(j,1)).^2 + (KidArray.Positions(i,2) - KidArray.Positions(j,2)).^2) < 2*KidArray.Radius) 
                    KidArray.Positions(i,:) = rand(2,1);
                    KidArray.Positions(i,1) = 0.1*Room.Width + KidArray.Positions(i,1) * 0.8*Room.Width;
                    KidArray.Positions(i,2) = 0.1*Room.Height + KidArray.Positions(i,2) * 0.8*Room.Height;
                    dist_KidKid = 0;
                    break;
                end
            end
            if (exit_for == 1)
                dist_KidKid = 0;
                break
            else 
                dist_KidKid = 1;
            end
        end
        
        % Check distance between balloons
        exit_for = 0;
        for i = 1:numBal
            for j = 1:numBal
                if i==j
                    continue;
                end 
                if ( sqrt( (BalloonArray.Positions(i,1) - BalloonArray.Positions(j,1)).^2 + (BalloonArray.Positions(i,2) - BalloonArray.Positions(j,2)).^2) < 2*BalloonArray.Edge) 
                    BalloonArray.Positions(i,:) = rand(2,1);
                    BalloonArray.Positions(i,1) = 0.1*Room.Width + BalloonArray.Positions(i,1) * 0.8*Room.Width;
                    BalloonArray.Positions(i,2) = 0.1*Room.Height + BalloonArray.Positions(i,2) * 0.8*Room.Height;
                    exit_for = 1;
                    break;
                end
            end
            if (exit_for == 1)
                dist_BalBal = 0;
                break
            else 
                dist_BalBal = 1;
            end
        end
        
        % Check distance between kids and balloons
        exit_for = 0;
        for i = 1:numKid
            for j = 1:numBal
                if ( sqrt( (KidArray.Positions(i,1) - BalloonArray.Positions(j,1)).^2 + (KidArray.Positions(i,2) - BalloonArray.Positions(j,2)).^2) < BalloonArray.Edge + KidArray.Radius) 
                    KidArray.Positions(i,:) = rand(2,1);
                    KidArray.Positions(i,1) = 0.1*Room.Width + KidArray.Positions(i,1) * 0.8*Room.Width;
                    KidArray.Positions(i,2) = 0.1*Room.Height + KidArray.Positions(i,2) * 0.8*Room.Height;
                    BalloonArray.Positions(j,:) = rand(2,1);
                    BalloonArray.Positions(j,1) = 0.1*Room.Width + BalloonArray.Positions(i,1) * 0.8*Room.Width;
                    BalloonArray.Positions(j,2) = 0.1*Room.Height + BalloonArray.Positions(i,2) * 0.8*Room.Height;
                    exit_for = 1;
                    break;
                end
            end
            if (exit_for == 1)
                dist_KidBal = 0;
                break
            else 
                dist_KidBal = 1;
            end
        end
end

% Random x and y velocities
KidArray.DesiredVel = rand(numKid,1)*(2.2-0.5) + 0.5;
KidArray.ActualVel = zeros(numKid,2);

% Figure 
figure(1)
axis equal
axis([0 Room.Width 0 Room.Height])
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
        'Curvature',[1 1], 'FaceColor', KidArray.Color(i,:));
end

squarefig = zeros(1,numBal);

for i = 1:numBal
    x_min_b = BalloonArray.Positions(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.Positions(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor', KidArray.Color(i,:));
    %BalloonArray(i);        
end


end