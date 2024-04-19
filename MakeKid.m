function Kid = MakeKid(x, y, xvel, yvel, radius, ID)
% Summary: Struct instantiation of a kid.
% Description: Creates a struct with all information needed for a kid.
% Kid is represented as a circle.
% Each kid has: x position, y position, x velocity, y velocity, 
%               radius, identification number.

Kid = struct('x',x,'y',y,'vx',xvel,'vy',yvel,'r',radius,'ID',ID);
end
