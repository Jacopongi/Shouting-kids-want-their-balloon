function Balloon = MakeBalloon(x, y, edge, ID)
% Summary: struct instantiation of a balloon.
% Description: create a struct with all information needed for a balloon.
% Balloon is represented as a square.
% Each balloon has: x position, y position, 
%                   edge lenght, identification number.
Balloon = struct('x',x,'y',y,'edge',edge,'ID',ID);
end