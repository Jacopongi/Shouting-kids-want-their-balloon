function [Result] = runrun(Case, subcase, distrpattern, numkids, numballs, trajectoryflag, videoflag) 

close all
          
%% Parameter initialization

% Number of Kids 
numKid = numkids;        
% Number of Balloon
numBal = numballs;

% Room features
MaxNum = numKid + numBal;
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]  

% Secret parameters that make the world correctly functioning
params.t = 0.5;
params.plotTrajEst = trajectoryflag;      % if 0 plot actual position
params.NumSFMExec = 0;
params.flagForce = 1;        % Activate/deactivate repulsive force of non-targeted balloons
params.print_flag = 1;
params.occupancyMap = zeros(5*Room.Height, 5*Room.Width); % NOTE: Rows are height
params.VideoFlag = videoflag;
params.frames = cell(1,1);
params.text = 1;
run = 1;

%% CASE SELECTION 

params.Case = Case;
params.Subcase = subcase;

%% Kids and balloons positioning inside the room

distrPattern = distrpattern;
[KidArray, BalloonArray, params] = distributeKidBalloon(numKid, numBal, Room, distrPattern, params);

%% Algorithm to automatically choose sensors number 

% Set the minimum number of sensors to consider
min_SensorNum = 5;
% Set the maximum number of sensors to consider
max_SensorNum = 15;

% Algorithm to choose sensors number
Sensor = ChooseSensorNumber(min_SensorNum, max_SensorNum, Room);

%% Testing positions estimation of kids and balloons

disp('Testing position estimate algorithms...')
disp(' ')
disp(['KIDS''' ' POSITION ESTIMATION TEST'])
KidArray.EstimatedPos = EstimatePosition(KidArray, Sensor, Room, params.text);
KidArray.EstimatedPosNC = EstimatePositionNoCons(KidArray, Sensor, Room);

disp(' ')
disp(['BALLOONS''' ' POSITION ESTIMATION TEST'])
BalloonArray.EstimatedPos = EstimatePosition(BalloonArray, Sensor, Room, params.text);
BalloonArray.EstimatedPosNC = EstimatePositionNoCons(BalloonArray, Sensor, Room);
disp(' ')

%% Simulation settings

disp('Here we are finally. Now we can free our impatient kids!')
disp(['The case chosen is ' num2str(params.Case) '.' num2str(params.Subcase)])
disp('3,2,1... Go!')
disp(' ')

% One copy for optimization, one for metrics
KidArrSFM = KidArray;
BalArrSFM = BalloonArray; 

%% Main section (Social Force Model)

while run
    % Call the Social Force Model
    [KidArrSFM, BalArrSFM, params] = ...
                    SFM2(KidArrSFM, BalArrSFM, Sensor, Room, params);

    % Call CaseDistinction to process data according to correct case
    [KidArrSFM, BalArrSFM, params] = ...
                    CaseDistinction(KidArrSFM, BalArrSFM, Sensor, Room, params);
                
    % Condition to exit loop
    if KidArrSFM.N == 0
        fprintf("\n");
        disp("Finish!!! What a run, boys!");
        run = 0;
    end
end


%% Obtain a video from animation 

if params.VideoFlag
    % capture last frame and duplicate it to make end of video nicer
    figure(1)
    frame = getframe(gcf);
    for i = 1:10
        params.frames{end+1} = frame;  
    end

    % prepare filename to prevent overwriting of previous recordings
    i = 1;
    filename = "./figures/Case" + num2str(params.Case) + num2str(params.Subcase) + ...
               "_" + num2str(distrPattern) + "_" + num2str(i) + ".avi";
    while isfile(filename)
        i = i+1;
        filename = "./figures/Case" + num2str(params.Case) + num2str(params.Subcase) + ...
                   "_" + num2str(distrPattern) + "_" + num2str(i) + ".avi";
    end

    v = VideoWriter(filename);   
    v.FrameRate = 15;                 
    open(v);                        

    % Write captured frames to VideoWriter
    for i = 1:length(params.frames)
        writeVideo(v, params.frames{i});
    end
    close(v);
end

%% Evaluation of the results 

PlotMetrics(KidArrSFM, KidArray);

Result.TravelTime = KidArrSFM.TravelTime;
Result.TotalTime = max(KidArrSFM.TravelTime); 
Result.PathLength = KidArrSFM.PathLength;
Result.AvgVel = KidArrSFM.PathLength ./ KidArrSFM.TravelTime;
Result.DesVel = KidArray.DesiredVel;


end
