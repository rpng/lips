% cursorLocation - WindowButtonMotionFcn displaying cursor location in plot
%===============================================================================
% Description : Display the current cursor location within the bounds of a 
%               figure window. Assigned as a WindowButtonMotionFcn callback 
%               function. Only updates when mouse is moved over plot contents.
% 
% Parameters  : obj         - Figure originating the callback
%               event       - not used (but required)
%               location    - Location within plot window for text. Can be
%                             'BottomLeft', 'BottomRight', 'TopRight', 'TopLeft'
%                             or a [1x2] array of XY location
%               format_str  - A sprintf format string that will accept 2 float 
%                             parameters. ie 'X: %.3f, Y: %.3f'
%               text_color  - either a color character (ie 'r') or a RGB
%                             triplet (ie [1.0 1.0 0.5])
%
% Return      : None
%
% Usage       : Assign to a Figure WindowButtonMotionFcn callback:
%                 set(fig_handle, 'WindowButtonMotionFcn', 
%                     @(obj, event)cursorLocation(obj, event, 'BottomLeft', 
%                     'X: %.3f, Y: %.3f', 'r')
%
% Author      : Rodney Thomson
%               http://iheartmatlab.blogspot.com
%===============================================================================                   
function cursorLocation(obj, event, location, format_str, text_color)

    % find axes associated with calling window
    axes_handle = findobj(obj, 'Type', 'axes');
    
    % Only support figures with 1 axes (Avoids confusion)
    if (isempty(axes_handle) || (numel(axes_handle) > 1)) 
        % None or too many axes to draw into
        return;
    end
    
    % Format the text string
    curr_point      = get(axes_handle, 'CurrentPoint');
    position        = [curr_point(1, 1) curr_point(1, 2)];
    position_string = sprintf(format_str, position);
    
    user_data = get(axes_handle, 'UserData');

    % If this is the first callback call, then will need to initialise the text
    % object and save to the axes in UserData
    if (~isfield(user_data, 'text_handle') || ~ishandle(user_data.text_handle))
        user_data.text_handle = text(nan, nan, '',         ...
                                    'Parent', axes_handle, ...
                                    'Color', text_color);
        set(axes_handle, 'UserData', user_data); % save to axes
    end
    
    % work out where we need to put the text
    [x_lims y_lims in_bounds] = getBounds(axes_handle, position);
    
    if (~in_bounds)
        % Not in the plot bounds so dont update
        return;
    end
    
    % Default to bottom left
    vertical_alignment   = 'bottom';
    horizontal_alignment = 'left';
    text_position        = [x_lims(1) y_lims(1)];
    
    if (ischar(location))
        switch location
            case 'BottomLeft'
                vertical_alignment   = 'bottom';
                horizontal_alignment = 'left';
                text_position        = [x_lims(1) y_lims(1)];
            case 'BottomRight'
                vertical_alignment   = 'bottom';
                horizontal_alignment = 'right';
                text_position        = [x_lims(2) y_lims(1)];
            case 'TopRight'
                vertical_alignment   = 'top';
                horizontal_alignment = 'right';
                text_position        = [x_lims(2) y_lims(2)];
            case 'TopLeft'
                vertical_alignment   = 'top';
                horizontal_alignment = 'left';
                text_position        = [x_lims(1) y_lims(2)];
            otherwise
                % Do nothing -> Default
        end
    elseif (isnumeric(location) && (numel(location) == 2))
        % Numeric... user sets text bottom left position
        text_position(1) = location(1);
        text_position(2) = location(2);
    end
    
    set(user_data.text_handle, 'Position', text_position,               ...
                               'String', position_string,               ...
                               'VerticalAlignment', vertical_alignment, ...
                               'HorizontalAlignment', horizontal_alignment);

end

%===============================================================================
% Description : Return the bounds of the supplied axes and identify whether
%               supplied position is contained within axes bounds
%===============================================================================
function [x_lims y_lims in_bounds] = getBounds(axes_handle, position)

    x_lims = get(axes_handle, 'XLim');
    y_lims = get(axes_handle, 'YLim');
    
    in_bounds = (position(1) >= x_lims(1)) && ...
                (position(1) <= x_lims(2)) && ...
                (position(2) >= y_lims(1)) && ...
                (position(2) <= y_lims(2));

end
