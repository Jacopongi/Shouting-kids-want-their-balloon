
%% SHOUTING KIDS WANT THEIR BALLOON
%  Course:        Intelligent Distributed Systems
%  Professor:     Fontanelli Daniele
%  Academic Year: 2023-2024
%  Students:      Endrizzi Jacopo  
%                 Pfluger Thade  
            
clc
clear
close all

disp('SHOUTING KIDS WANT THEIR BALLOON')
disp('A project by Jacopo Endrizzi and Thade Pfluger')
disp(' ')
disp('Oh, come on. We are not going to be so serious...')
disp(' ')

%% Parameter initialization


% Number of Kids 
numKid = 8;        
% Number of Balloon
numBal = 8;


% NOTE1: choose an equal number of kids and balloons!
% You want to make sure that everyone is happy at the end, right?

% NOTE2: Yes, of course we have a section for playing with different numbers of
% kids and balloons, but for the moment we try to be organized. 
% If you are just interested in games, go to last section.

% NOTE3: for visualization ease, we suggest staying below 15 with kids and balloons.
% Otherwise, be prepared for the mess...

% NOTE4: there is always a note 4, but in this case is useless.

% Room features
MaxNum = numKid + numBal;
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m]  

% Secret parameters that make the world correctly functioning
params.t = 0.5;              % step size          
params.plotTrajEst = 1;      % if 0 plot actual position
params.NumSFMExec = 0;
params.flagForce = 1;        % Activate/deactivate repulsive force of non-targeted balloons
params.print_flag = 1;
params.occupancyMap = zeros(5*Room.Height, 5*Room.Width); % NOTE: Rows are height
params.VideoFlag = 1;
params.frames = cell(1,1);
params.text = 1;
run = 1;


%% CASE SELECTION 

% Case = [1]    Subcase = [1]   --> Full knowledge, actual positions   
%               Subcase = [2]   --> Full knowledge, estimated positions
% 
% Case = [2]    Subcase = [1]   --> Restricted knowledge, no collaboration
%               Subcase = [2]   --> Restricted knowledge, collaboration
% 
% Case = [3]    Subcase = [1]   --> Random exploration, no collaboration   
%               Subcase = [2]   --> Random exploration, collaboration
% 
% Case = [4]    Subcase = [1]   --> Exploration with occupancy map   

params.Case = 4;
params.Subcase = 1;

%% Kids and balloons positioning inside the room

% Choose distribution: distrPattern =  [1] -> Random distribution
%                                      [2] -> Grid-Grid distribution
%                                      [3] -> Grid-Arc distribution 
%                                      [4] -> Circular distribution
%                                      [5] -> Block-Triangle distribution
distrPattern = 1;
[KidArray, BalloonArray, params] = distributeKidBalloon(numKid, numBal, Room, distrPattern, params);

disp('Have you seen that little kid right in the corner?')
disp(' ')
disp('Okay. Some burocratic work before real fun...')
disp(' ')

%% Sensors deployment

% Choose the number of sensors to distribute around the room perimeter 
Sensor.Num = 7;

disp(['You have chosen a set of ', num2str(Sensor.Num), ' sensors to consider.'])
disp('Alright. Distributing sensors around the perimeter...')
% For plots display, set: Plot = true
PlotSens = true;
Sensor.Position = distributeSensorsOnPerimeter(Sensor.Num, Room.Width, Room.Height, PlotSens);

disp('Done. Sensors are there. They seem so well distributed!')
disp(' ')

%% Algorithm to automatically choose sensors number 

% This method tries to find the set that minimizes the maximum error 
% in estimation of several reference positions.

disp('It is so annoying choosing the sensor number!')
disp('Now we automatize this step...')

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

%% Testing sensors: shared VS single estimate

% Set dimension of the grid of positions to be tested
% NOTE: use GridDim > 50 to clearly see results
GridDim = 100; 

disp(['One position is okay. But what about ' num2str(GridDim^2) ' positions?' ])
disp('We verify if shared estimates are better than single ones!')
disp('Starting a bigger test...')

Test = TestEstimatePosition(Sensor, Room, GridDim);
Test = TestEstimatePositionNoCons(Sensor, Room, Test, GridDim);
TestEstimateComparison(Test)
disp(['Done. You don''' 'want me to redo it, do you? Plese no, no...'])
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

% Pay attention: model is going to make a lot of noise...

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

% Technical section to make you feel like a well-renowned film director.

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
    v.FrameRate = 10;                 
    open(v);                        

    % Write captured frames to VideoWriter
    for i = 1:length(params.frames)
        writeVideo(v, params.frames{i});
    end
    close(v);
end

%% Evaluation of the results

% Some metrics for experts that want to discuss performances
%   - traveled path length
%   - overall travel time 

PlotMetrics(KidArrSFM, KidArray);

%% PLAYROOM 

% After all this work, some games to relax. 
% Yes, we know this is the most relevant part of the project.

%% ***** FLOCKING GAME ***** %%

% Here you can see how flocking behavior works.
% NOTE: It is allowed and highly recommended to play with the numbers 
% and the different distributions of kids and balloons.

% Number of Kids
numKid = 5;
% Number of Balloon
numBal = 5;

% Room features
MaxNum = numKid + numBal;
Room.Width = MaxNum * 1.5;  % [m]                     
Room.Height = MaxNum ;      % [m] 

% Choose your favorite distribution
% distrPattern =  [1] -> Random distribution
%                 [2] -> Grid-Grid distribution
%                 [3] -> Grid-Arc distribution 
%                 [4] -> Circular distribution
%                 [5] -> Block-Triangle distribution

distrPattern = 4;
[KidArray, BalloonArray, params] = distributeKidBalloon(numKid, numBal, Room, distrPattern, params);

% To enable Flocking behavior set FlockingFlag = 1
% if FlockingFlag = 0, only collision avoidance is enabled. 
FlockingFlag = 1;
% To create the video of the animation, put VideoFlag = 1
VideoFlag = 0;

[KidArray, BalloonArray] = Flocking(KidArray,BalloonArray,Room, FlockingFlag, VideoFlag);

% NOTE: There is a maximum number of iterations because also patience has limits.


%% ***** RENDEZVOUS GAME ***** %%

% Here you can try how rendezvous consensus algorithm works.
% After the first run, you can pick your favorite position inside the room...
% ... and robots will converge there!

RendezvousGame

%% THAT'S ALL FOLKS!
disp(' ')
disp(['THAT''' 'S ALL FOLKS!'])
