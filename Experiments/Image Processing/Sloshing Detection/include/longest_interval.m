function [ind1, ind2] = longest_interval(BW,type)
% Inputs:
    % BW    - Array,    
              % For an image, scan column pixel from top to bottom
    % type  - 0 or 1, black or white
    
    % Output:
    % ind1 - first index
             % for an image, finds the first "type" pixel from top 
    % ind2 - last index
             % for an image, finds the last "type" pixel from top


% find all the rows in which pixel of the column is "type" (returns vector)
ind = find(BW==type);

if isempty(ind) || length(ind) == 1
    ind1 = [];
    ind2 = [];
else

    df = diff(ind); % calculates differences between adjacent elements 
    n = length(df);
    j = 1;
    interval = [];
    interval_length = [];
    for i = 1:n

        if df(i) == 1
            interval = [interval ind(i)];
            str(j).interval = interval;
        else
            if i == 1
                j = 1;
            elseif df(i-1) == 1
                interval_length = [interval_length (length(str(j).interval)+1)];
                j = j+1;
                interval = [];
            end
        end

    end

    if df(end) == 1
        interval_length = [interval_length (length(str(j).interval))];
    end

    if isempty(interval_length)
        ind1 = [];
        ind2 = [];
    else
        [max_int, ind_maxint] = max(interval_length);

        ind1 = str(ind_maxint).interval(1); 
        ind2 = ind1 + max_int - 1;
    end
end

end