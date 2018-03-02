function [ hits, polyids, rayids ] = kevinsPlaneIntersection( rays, polys )
%KEVINSPLANEINTERSECTION Will try to intersect all rays with all polygons
%


% Setup our return values
hits = [];
polyids = [];
rayids = [];

% Debug info
fprintf('RAY: Intersecting %d rays to %d planes...\n',size(rays,2),size(polys,2))


% Next up, lets try to intersect each ray with a polyg
for ii=1:size(rays,2)
    for jj=1:size(polys,2)
        % Try to intersect with this polygon
        M = [-rays{ii}(4:6)', polys{jj}(2,:)'-polys{jj}(1,:)', polys{jj}(3,:)'-polys{jj}(2,:)'];
        solution = M\(rays{ii}(1:3)'-polys{jj}(1,:)');
        % Check for success
        if solution(1) >= 0 && solution(2) >= 0 && solution(2) <= 1 && solution(3) >= 0 && solution(3) <= 1
            % Calculate the point and append it
            pointING = rays{ii}(1:3)+solution(1)*rays{ii}(4:6);
            hits = [hits; pointING];
            % Record the id
            polyids = [polyids; jj];
            rayids = [rayids; ii];
        end
    end
end


% Debug info
fprintf('RAY: %d ray-polygon intersections!\n',size(hits,1))


end

