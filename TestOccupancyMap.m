function [nextPos, params] = TestOccupancyMap(KidArrSFM, Room, params)
% Summary: function that guides the robot flock to the still unexplored areas
% How could this work with sensors in a real system??

% mapSize = 10*[Room.Width, Room.Height];
% 
% % Initialize the occupancy map with zeros
% params.occupancyMap = zeros(mapSize);

posX = round(5 * KidArrSFM.ActualPos(:,1));
posY = round(5 * KidArrSFM.ActualPos(:,2));

% if params.NumSFMExec == 2 % only needed once, here 2 is the first call to this function
%     % set boundaries of room to 1
%     params.occupancyMap(:,1:4) = 1;
%     params.occupancyMap(1:4,:) = 1;
%     params.occupancyMap(:,end-4:end) = 1;
%     params.occupancyMap(end-4:end,:) = 1;
% end

% Mark the positions as visited
for i = 1:KidArrSFM.N
    params.occupancyMap(posY(i), posX(i)) = 1;  
end

% Display the updated occupancy map
figure(101)
imagesc(params.occupancyMap);
set(gca, 'YDir', 'normal'); % Ensure the y-axis rises from bottom to top
title("Occupancy Map")


%% Sliding window method to determine 'density' of un-/explored areas
% old try 
% windowSize = [Room.Height, Room.Width];
% [totalHeight, totalWidth] = size(params.occupancyMap);
% densityMap = zeros(totalHeight,totalWidth);
%
% for i = 1:windowSize(1):totalHeight - windowSize(1) + 1
%     for j = 1:windowSize(2):totalWidth - windowSize(2) + 1
%         window = params.occupancyMap(i:i+windowSize(1)-1, j:j+windowSize(2)-1);
%         densityMap(i,j) = sum(window(:));
%     end
% end
%
% windowSize = size(params.occupancyMap)/5;  % = [Room.Height, Room.Width];
% [totalHeight, totalWidth] = size(params.occupancyMap);
% densityMap = zeros(totalHeight/Room.Height,totalWidth/Room.Width);
% 
% for i = 1:5
%     for j = 1:5
%         window = params.occupancyMap((i-1)*windowSize(1)+1:i*windowSize(1), (j-1)*windowSize(2)+1:j*windowSize(2));
%         densityMap(i,j) = sum(window(:));
%         % % group them in steps of 10
%         % a = floor(densityMap(i,j)/10);
%         % b = mod(densityMap(i,j),10);
%         % densityMap(i,j) = densityMap(i,j)/b * a;
%     end
% end
%
% figure
% imagesc(densityMap);
% set(gca, 'YDir', 'normal'); % Ensure the y-axis rises from bottom to top
% title("Density map of visited positions")
%
% [row,col] = find(densityMap <= 8);
% 
% % get center of the areas of density map
% centers = zeros(length(col),2);
% for k = 1:length(col)
%     centers(k,1) = ((col(k)-0.5) * Room.Width)/5;  % cols are width!
%     centers(k,2) = ((row(k)-0.5) * Room.Height)/5;  % rows are height!
% end
% 
% % Distance between CoM of kids in flock and the next candidates
% flockPos = mean(KidArrSFM.ActualPos(find(ismember(KidArrSFM.ID,find(KidArrSFM.FlagPosReceived==0))),:),1);
% [~, idxDist] = min(pdist2(centers, flockPos));
% 
% % Previous destination/targeted unexplored area
% prevPos = mean(KidArrSFM.Destinations(find(ismember(KidArrSFM.ID,find(KidArrSFM.FlagPosReceived==0))),:),1);
% % d_prev = pdist2(prevPos, flockPos);
% 
% % keep targeting the previous position if not yet explored by at least one kid
% if ~any(find(all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[2 2],2)))
%     nextPos = prevPos;
% else
%     % go to next closest of the 20 candidate areas once the old area has been visited
%     nextPos = centers(idxDist,:);
% end
% 
% 
% 
%
% lsm = zeros();
% for i=1:10
%     for j=1:10
%         if
%     % largest sized matrix
%     lsm(i,j) = min([lsm(i-1, j), lsm(i, j-1), lsm(i-1, j-1)]) + 1;
%     end
% end
% 


%% Find the largest unvisited block
%  ...and set its center as the new target position for the flock  

% Previous destination/targeted unexplored area
prevPos = mean(KidArrSFM.Destinations(find(ismember(KidArrSFM.ID,find(KidArrSFM.FlagPosReceived==0))),:),1);

% keep targeting the previous position if not yet explored by at least one kid
if ~any(find(all(abs(KidArrSFM.ActualPos - KidArrSFM.Destinations)<[1 1],2)))
    nextPos = prevPos;
else
    % choose next closest area once the old one has been visited    
    maxSize = zeros(1,5);  % save the five largest
    maxPos = zeros(5,2);   
    
    % Create a matrix to store the size of the largest square sub-matrix ending
    % at (i,j), which means (i,j) is its top right corner
    lsm = zeros(size(params.occupancyMap));
    % max_idx = 1;
    for i = 1:5*Room.Height
        for j = 1:5*Room.Width
            if (i == 1) || (j == 1)
                lsm(i,j) = 1;
            elseif params.occupancyMap(i, j) == 0
                % avoids the field where occMap = 1 --> respective field in
                % lsm correctly remains zero 
                currentSize = min([lsm(i-1, j), lsm(i, j-1), lsm(i-1, j-1)]);
                lsm(i,j) = currentSize + 1;
    
                % if lsm(i, j) > min(maxSize)
                %     % Find the index of the smallest and biggest size in maxSize
                %     [~, min_idx] = min(maxSize);
                %     % [~, max_idx] = max(maxSize);
                % 
                %     if any(pdist2([i,j],maxPos) < 20) 
                %         % check if there's already a big area closeby
                %         max_idx = find(pdist2([i,j],maxPos)<20,1);
                %         if (lsm(i,j) > maxSize(max_idx))
                %             % overwrite/enlarge the locally biggest area
                %             maxSize(max_idx) = lsm(i, j);
                %             maxPos(max_idx, :) = [i, j];
                %         end
                %     else%if (abs(i-maxPos(max_idx,1)) > 50) || ...
                %         %   (abs(j-maxPos(max_idx,2)) > 50)
                %         % Update the smallest entry with the new candidate
                %         maxSize(min_idx) = lsm(i, j);
                %         maxPos(min_idx, :) = [i, j]; 
                %         % max_idx = max_idx + 1;
                %     end
                % end
            end
        end
    end
    
    % find the biggest area, save it, overwrite it, find 2nd biggest, repeat
    for i = 1:5
        maxSize(i) = max(lsm,[],"all");
        [r,c] = find(lsm == maxSize(i));
        % only take the first one, if there are more with same size
        r_uni = unique(r);
        c_uni = unique(c);
        % make sure to overwrite all by taking the top-right point of rectangle
        % as reference index
        if length(r_uni)>1 && length(c_uni)==1
            r_max = r_uni(end);   % top most row
            c_max = c_uni;
        elseif length(c_uni)>1 && length(r_uni)==1
            c_max = c_uni(end); % right most column
            r_max = r_uni;
        elseif length(c_uni) > length(r_uni)
            r_max_idx = find(ismember(r,r_uni(1)));
            c_max = c(r_max_idx(end)); % right most column
            r_max = r(r_max_idx(end));
        elseif length(c_uni) < length(r_uni)
            c_max_idx = find(ismember(c,c_uni(1)));
            r_max = r(c_max_idx(end));    % top most row
            c_max = c(c_max_idx(end));
        else 
            % only one rect of maxSize spotted. Still there could be >1 entries
            % of same size of the same rectangle => take the last bc they're
            % sorted => automatically the top-right one
            r_max = r(end);
            c_max = c(end);
        end    
        maxPos(i,:) = [r_max,c_max];  % end in case of a rectangle
        lsm(r_max-maxSize(i)+1:r_max, c_max-maxSize(i)+1:c_max) = 0;
    end
    
    
    
    % Calculate the middle points of the largest square candidates
    candidates = zeros(5,2);
    for k = 1:5
        candidates(k,1) = (maxPos(k,2) - floor(maxSize(k)/2))/5;  % rows are height!
        candidates(k,2) = (maxPos(k,1) - floor(maxSize(k)/2))/5;
    end
    
    % Distance between CoM of kids in flock and the next candidates
    flockPos = mean(KidArrSFM.ActualPos(find(ismember(KidArrSFM.ID,find(KidArrSFM.FlagPosReceived==0))),:),1);
    [~, idxDist] = sort(pdist2(candidates, flockPos));
        % idxDist(1) now contains the index of the closest candidate
    
    % remember that area sizes are already sorted, 1 biggest, 2 second biggest, ...
    % => currently candidate(1) contains largest
    weight_size = (1:5)';
    % create weight to decide for next position
    weight_dist = zeros(5,1);
    for i = 1:5
        weight_dist(idxDist(i)) = i;
    end
    weight = weight_size + weight_dist;
    [~,idx] = min(weight);
    nextPos = candidates(idx,:);
    disp(['Now, we want to go to [' num2str(nextPos(1)) ' ' num2str(nextPos(2)) ']!'])
end

end
