function [ data ] = load_2dplanedata( path )
%LOAD_2DPLANEDATA Loads a txt file with 2d lines in it representing planes
%   Given a path this will load the contents into a simple matrix of those
%   generated 2D lines. Theses lines are normally a floorplan of a
%   building.

% Matrix we will append to
data = [];

% Open our text file
fid = fopen(path);

% Debug info
disp('DATA: Opening file....')

% Get the first line of the file
tline = fgetl(fid);
while ischar(tline)
    % Read the line if there is content on it
    % And as long as it is not a comment line
    if size(tline,1) == 1 && tline(1,1) ~= '#' && tline(1,1) ~= ' '
        % Convert to double matrix
        valuesstr = strsplit(tline,'\s*,*','DelimiterType','RegularExpression');
        valuesdb = str2double(valuesstr);
        % Append to our list if we have the stat and end points
        if size(valuesdb,1) == 1 && size(valuesdb,2) == 4        
            data = [data; valuesdb];
        end 
    end
    % Get the next line
    tline = fgetl(fid);
end

% Close the file
fclose(fid);

% Debug info
disp('DATA: Done, file closed.')

end

