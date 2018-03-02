function [ hitsclean, polyidsclean, rayidsclean ] = invalidateintersections( pos_IMUinG, hits, polyids, rayids )
%INVALIDATEINTERSECTIONS Will handle that a single ray hits many planes
%   We will go through each rayid and from there we will check to see which
%   of the given intersections for that ray are closest to the given
%   position of the LIDAR. We know that the closest ray would intersect
%   first so that is the only valid ray intersection.


% Sort the arrays based on the rayid
[rayids, indices] = sort(rayids);
hits = hits(indices,:);
polyids = polyids(indices,:);

% Array to keep track of the max distance from our pos
maxdist = inf(1,max(rayids));
maxids = zeros(1,max(rayids));

% Now we know we are sorted, so lets loop through each one and find the
% shortest distance for the given ray ids
for ii=1:size(rayids,1)
    % compute the square difference between this point and the LIDAR pose
    diff = sqrt(sum((pos_IMUinG-hits(ii,1:3)).^2));
    % next lets see if it is the new min  
    if diff < maxdist(rayids(ii,1))
        maxdist(rayids(ii,1)) = diff;
        maxids(rayids(ii,1)) = ii;
    end
end


% our output args
hitsclean = [];
polyidsclean = [];
rayidsclean = [];


% finally lets create the return values
cleanct = 0;
for ii=1:size(maxdist,2)
    % if it is not inf then we have the min
    if maxdist(1,ii) ~= Inf
        hitsclean = [hitsclean; hits(maxids(1,ii),:)];
        polyidsclean = [polyidsclean; polyids(maxids(1,ii),:)];
        rayidsclean = [rayidsclean; rayids(maxids(1,ii),:)];
        cleanct = cleanct + 1;
    end    
end


% Debug info
fprintf('RAY: %d left after invalidation.\n',cleanct)














end

