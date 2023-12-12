function Kid = MakeKid(x, y, xvel, yvel, radius, ID)
% MakeKid: Creates a moving circle representing a kid
%          Each kid has: x position, y position, x velocity, y velocity, 
%                        radius, identification number 
Kid = struct('x',x,'y',y,'vx',xvel,'vy',yvel,'r',radius,'ID',ID);
end
