function [inside,wall] = isInside(px,py,Room)
% Summary: short function to check if a point lies inside
%          the rooms boundaries or not

top = (py <= Room.Height-2.1);
right = (px <= Room.Width-2.1);
bottom = (py >= 2.1);
left = (px >= 2.1);

inside = top && right && bottom && left;

if ~inside
    if ~top
        wall = "top";
    elseif ~right
        wall = "right";
    elseif ~bottom
        wall = "bottom";
    elseif ~left
        wall = "left";
    end
else
    wall = "";
end
end