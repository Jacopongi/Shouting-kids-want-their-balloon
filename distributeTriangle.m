function points = distributeTriangle(A, B, C, n)
    
    points = [];
    % Points per edge
    pointsPerEdge = floor(n / 3);
    remainingPoints = mod(n, 3);
    
    % Points on edge AB
    for i = 1:pointsPerEdge
        t = i / (pointsPerEdge + 1);
        points = [points; (1 - t) * A + t * B];
    end
    
    % Points on edge BC
    for i = 1:pointsPerEdge
        t = i / (pointsPerEdge + 1);
        points = [points; (1 - t) * B + t * C];
    end
    
    % Points on edge CA
    for i = 1:pointsPerEdge
        t = i / (pointsPerEdge + 1);
        points = [points; (1 - t) * C + t * A];
    end
    
    % Remaining points to vertices
    if remainingPoints > 0
        vertexPoints = [A; B; C];
        points = [points; vertexPoints(1:remainingPoints, :)];
    end
end
