function [ hits, polyids, rayids ] = intersectrayspolys( rays, polys )
%INTERSECTRAYSPOLYS Will try to intersect all rays with all polygons
%   Will intersect each ray with each other polygon. It will not record
%   any information if the ray never hits anything. A hits array containing
%   the 3D point on the plane and the plane ID information is returned. The
%   IDs contain the ray and then the plane ID number for lookup.


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
        [inter, inside] = intersectRayPolygon3d(rays{ii}, polys{jj});
        % Check for success
        if inside
            hits = [hits; inter];
            polyids = [polyids; jj];
            rayids = [rayids; ii];
        end
    end
end


% Debug info
fprintf('RAY: %d ray-polygon intersections!\n',size(hits,1))


end

