function [KidArray, BalloonArray, params] = distributeKidBalloon(numKid, ...
                                        numBal, Room, DistrType, params)
% Summary: random distribution of a set of agents and objectives in a room.
% Description: after the instation of the main characteristics of agents and objectives,
% they are randomly distributed in the room paying attention to avoid walls and overlaps. 
% Plots are available for final visualization.

% Distribution Type (DistrType Flag): [1] - Random Distribution
%                                     [2] - Grid-Grid Distribution
%                                     [3] - Grid-Arc Distribution 
%                                     [4] - Circular Distribution

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
KidArray.OldPos = zeros(numKid, 2); 
KidArray.OldVel = zeros(numKid,2);
KidArray.InitCond = zeros(4*numKid,1);
BalloonArray.InitPos = zeros(numBal, 2);
% - for the plots
KidArray.circlefig = zeros(1,numKid);
KidArray.text = zeros(1,numKid);
BalloonArray.squarefig = zeros(1,numBal);
BalloonArray.plotBalID = zeros(1,numBal);
% - remember ID's (row where ID is saved) of kids that have arrived
KidArray.ID_arr = (1:numKid)';
BalloonArray.ID_arr = (1:numBal)';
% Only case 2:
% - remember which balloons have been visited by each kid
KidArray.BalVisited = zeros(numKid);
% - remember which kids have been informed about the location of their balloon
% KidArray.MemoPosReceived = zeros(numKid,1);
KidArray.FlagPosReceived = zeros(numKid,1);


% ID
KidArray.ID = (1:numKid)';
BalloonArray.ID = (1:numBal)';

% Bounds on velocity

maxVel = 2;%2.2;   % [m/s]     
minVel = 1.5;%0.5;   % [m/s]   

% Positions estimated by sensors 
KidArray.EstimatedPos = zeros(numKid,2); 

if(DistrType == 1) 

        % RANDOM Starting Positions
        KidArray.ActualPos = rand(numKid,2);
        KidArray.Destinations = zeros(numKid,2);
        BalloonArray.ActualPos = rand(numBal,2);
        
        ScaleFactor = 0.1; % Note! Between 0 and 1!
                           % Needed to avoid generation on the walls
        
        KidArray.ActualPos(:,1) = ScaleFactor * Room.Width + KidArray.ActualPos(:,1) * (1- 2*ScaleFactor) * Room.Width;
        KidArray.ActualPos(:,2) = ScaleFactor * Room.Height + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;
        
        BalloonArray.ActualPos(:,1) = ScaleFactor * Room.Width + BalloonArray.ActualPos(:,1) * (1-2*ScaleFactor) * Room.Width;
        BalloonArray.ActualPos(:,2) = ScaleFactor * Room.Height + BalloonArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;
        
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
                
                KidArray.ActualPos(:,1) = ScaleFactor * Room.Width + KidArray.ActualPos(:,1) * (1-2*ScaleFactor) * Room.Width;
                KidArray.ActualPos(:,2) = ScaleFactor * Room.Height + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;
                
                BalloonArray.ActualPos(:,1) = ScaleFactor * Room.Width + BalloonArray.ActualPos(:,1) * (1-2*ScaleFactor) * Room.Width;
                BalloonArray.ActualPos(:,2) = ScaleFactor * Room.Height + BalloonArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;
            else 
                OkayDistance = 1;
            end
        end

elseif(DistrType == 2)

        % GRID Starting Positions
        gridnum = 5;
        KFullColumnNum = fix(KidArray.N / gridnum);
        KRem = rem(KidArray.N, gridnum);
        KFullxPos = linspace(0.1, 0.1*KFullColumnNum, KFullColumnNum );
        KFullyPos = linspace(0.1, 0.9, gridnum);
        KRemxPos = linspace(0.1*(KFullColumnNum+1), 0.1*(KFullColumnNum+1), 1);
        KRemyPos = linspace(0.1, 0.9, KRem);
        KFullPos = table2array(combinations(KFullxPos, KFullyPos));
        KRemPos = table2array(combinations(KRemxPos, KRemyPos));
        KidArray.ActualPos = cat(1, KFullPos, KRemPos);
        
        BFullColumnNum = fix(BalloonArray.N / gridnum);
        BRem = rem(BalloonArray.N, gridnum);
        BFullxPos = linspace( (1-0.1*BFullColumnNum), 0.95, BFullColumnNum );
        BFullyPos = linspace(0.05, 0.95, gridnum);
        %BRemxPos = linspace( (1-0.1*(BFullColumnNum+1)), (1-0.1*(BFullColumnNum+1)), 1);
        if (BFullColumnNum == 1)
            BRemxPos = 0.95 - 0.15 * BFullColumnNum;
        else
            BRemxPos = 0.95 - diff(BFullxPos) * BFullColumnNum;
        end
        BRemyPos = linspace(0.05, 0.95, BRem);
        BFullPos = table2array(combinations(BFullxPos, BFullyPos));
        BRemPos = table2array(combinations(BRemxPos, BRemyPos));
        BalloonArray.ActualPos = cat(1, BFullPos, BRemPos);

        ScaleFactor = 0.1; % Note! Between 0 and 1!
                           % Needed to avoid generation on the walls
        
        KidArray.ActualPos(:,1) = ScaleFactor * Room.Width + KidArray.ActualPos(:,1) * (1- 2*ScaleFactor) * Room.Width;
        KidArray.ActualPos(:,2) = ScaleFactor * Room.Height + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;
        
        BalloonArray.ActualPos(:,1) = ScaleFactor * Room.Width + BalloonArray.ActualPos(:,1) * (1-2*ScaleFactor) * Room.Width;
        BalloonArray.ActualPos(:,2) = ScaleFactor * Room.Height + BalloonArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;

elseif(DistrType == 3)
        
        % GRID Starting Positions
        gridnum = 5;
        KFullColumnNum = fix(KidArray.N / gridnum);
        KRem = rem(KidArray.N, gridnum);
        KFullxPos = linspace(0.1, 0.1*KFullColumnNum, KFullColumnNum );
        KFullyPos = linspace(0.1, 0.9, gridnum);
        KRemxPos = linspace(0.1*(KFullColumnNum+1), 0.1*(KFullColumnNum+1), 1);
        KRemyPos = linspace(0.1, 0.9, KRem);
        KFullPos = table2array(combinations(KFullxPos, KFullyPos));
        KRemPos = table2array(combinations(KRemxPos, KRemyPos));
        KidArray.ActualPos = cat(1, KFullPos, KRemPos);

        ScaleFactor = 0.1; % Note! Between 0 and 1!
                           % Needed to avoid generation on the walls

        KidArray.ActualPos(:,1) = ScaleFactor * Room.Width + KidArray.ActualPos(:,1) * (1- 2*ScaleFactor) * Room.Width;
        KidArray.ActualPos(:,2) = ScaleFactor * Room.Height + KidArray.ActualPos(:,2) * (1-2*ScaleFactor) * Room.Height;

        % ARC Positions
        ab = linspace(-0.5*pi, 0.5*pi, BalloonArray.N);
        BalloonArray.ActualPos = zeros(numBal,2);

        rb = 0.15*Room.Width + 0.15*Room.Height;
        xb = rb*cos(ab) + 0.6*Room.Width;
        yb = rb*sin(ab) + 0.5*Room.Height;

        for i=1:BalloonArray.N
            BalloonArray.ActualPos(i,:) = [xb(i), yb(i)];
        end


elseif(DistrType == 4)
       
        ak = linspace(0, 2*pi, KidArray.N+1);
        KidArray.ActualPos = zeros(numKid,2);
        BalloonArray.ActualPos = zeros(numBal,2);

        rk = 0.1*Room.Width + 0.1*Room.Height;
        xk = rk*cos(ak) + Room.Width/2;
        yk = rk*sin(ak) + Room.Height/2;

        for i=1:KidArray.N
            KidArray.ActualPos(i,:) = [xk(i), yk(i)];
        end

        ab = linspace(0.5*pi, 2.5*pi, BalloonArray.N+1);
        rb = Room.Width + Room.Height;
        xx = rb*cos(ab) + Room.Width/2;
        yy = rb*sin(ab) + Room.Height/2;
        
        xlimit = [0.1*Room.Width 0.9*Room.Width];
        ylimit = [0.1*Room.Height 0.9*Room.Height];
        xbox = xlimit([1 1 2 2 1]);
        ybox = ylimit([1 2 2 1 1]);
        
        for i = 1:BalloonArray.N
            x_aux = [Room.Width/2 xx(i)];
            y_aux = [Room.Height/2 yy(i)];
            [xi, yi] = intersections(x_aux, y_aux, xbox, ybox, 1);
            BalloonArray.ActualPos(i,:) = [xi yi];
        end

end


% Save initial positions
KidArray.InitPos = KidArray.ActualPos;
KidArray.OldPos = KidArray.InitPos;
BalloonArray.InitPos = BalloonArray.ActualPos;

% Random x and y velocities
% KidArray.DesiredVel = rand(numKid,1)*(maxVel-minVel) + minVel;
KidArray.DesiredVel = rand(numKid,1)*(maxVel-minVel) + minVel;
KidArray.ActualVel = zeros(numKid,2);

% Figure 
figure(1)
axis equal
axis([-0.05*Room.Width 1.05*Room.Width -0.05*Room.Height 1.05*Room.Height])
rectangle('Position', [0 0 Room.Width Room.Height])
box on
hold on
xlabel('X Position');
ylabel('Y Position');
title('Shouting kids want their balloon');
subtitle(append('Case: ',num2str(params.Case),'.',num2str(params.Subcase)));


KidArray.Color = rand(numKid,3);
% if params.plotTrajEst
    % plot initial positions bigger. those will stay forever to mark
    % beginning of trajectory
    % if not true, the actual positions are plotted to visualize the
    % flocking behavior (see SFM2) => nicer for a video
    for i = 1:numKid
        x_min = KidArray.ActualPos(i,1) - KidArray.Radius;
        y_min = KidArray.ActualPos(i,2) - KidArray.Radius;
        radius_cur = KidArray.Radius;
        KidArray.circlefig(i) = rectangle('Position',[x_min,y_min,2*radius_cur,2*radius_cur],...
            'Curvature',[1 1], 'FaceColor',KidArray.Color(i,:));
        KidArray.text(i) = text(KidArray.ActualPos(i,1), KidArray.ActualPos(i,2), num2str(KidArray.ID(i)), ...
            'HorizontalAlignment', 'center', 'Color','k', 'FontSize', KidArray.Radius*15);
    end

% else
%     % doesn't matter bc we'd delete them right after plotting anyway
% end

% initial plot of balloon position is done in both plot cases
for i = 1:numBal
    x_min_b = BalloonArray.ActualPos(i,1) - 0.5*BalloonArray.Edge; 
    y_min_b = BalloonArray.ActualPos(i,2) - 0.5*BalloonArray.Edge;
    x_max_b = BalloonArray.Edge;
    y_max_b = BalloonArray.Edge;
    if(i > height(KidArray.Color))
        bcolor = rand(1,3);
        BalloonArray.squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor',bcolor);
    else 
        BalloonArray.squarefig(i) = rectangle('Position',[x_min_b y_min_b x_max_b y_max_b], ...
        'FaceColor','w');       % keep them secret/white at first,   KidArray.Color(i,:));
    end

    BalloonArray.plotBalID(i) = text(BalloonArray.ActualPos(i,1), BalloonArray.ActualPos(i,2), num2str(BalloonArray.ID(i)), ...
        'HorizontalAlignment', 'center', 'Color','k', 'FontSize', BalloonArray.Edge*10, 'Visible','off');

end

% Save frames for the video
if params.VideoFlag
    frame = getframe(gcf);   % Capture the current frame 
    params.frames{1} = frame;   
end

end
