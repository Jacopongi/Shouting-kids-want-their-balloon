function [inside,wall] = isInside(px,py,Room)
% short function to check if a point lies inside
% the rooms boundaries or not
if length(px) == 1
    top =    (py >= 0.9*Room.Height);
    right =  (px >= Room.Width - 0.1*Room.Height);
    bottom = (py <= 0.1*Room.Height);
    left =   (px <= 0.1*Room.Height);
    
    
    outside = top || right || bottom || left;
    inside = ~outside;
    
    if outside
        if top && right
            wall = "top-right";
        elseif right && bottom
            wall = "bottom-right";
        elseif bottom && left
            wall = "bottom-left";
        elseif left && top
            wall = "top-left";
        elseif top
            wall = "top";
        elseif right
            wall = "right";
        elseif bottom
            wall = "bottom";
        elseif left
            wall = "left";
        end
    else
        wall = "";
    end

% Case 3.2 - check all 7 possible new positions simultaneously
elseif length(px) > 1    
    top =    (py >= 0.9*Room.Height);
    right =  (px >= Room.Width - 0.1*Room.Height);
    bottom = (py <= 0.1*Room.Height);
    left =   (px <= 0.1*Room.Height);

    outside = top | right | bottom | left;

    inside = ~outside;
    wall = "";  % only for return values, not important


end