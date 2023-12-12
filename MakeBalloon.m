function Balloon = MakeBalloon(x, y, edge, ID)
% MakeBalloon: Creates a square representing a kid
%              Each balloon has: x position, y position, 
%                                edge lenght, identification number   
Balloon = struct('x',x,'y',y,'edge',edge,'ID',ID);
end