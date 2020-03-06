% clc, 
% [row, column] = t_findArrayToMatrixIndex(1, 3) % should return (1, 1)
% [row, column]= t_findArrayToMatrixIndex(3, 3) % should return (1, 3)
% [row, column]= t_findArrayToMatrixIndex(5, 3) % should return (2, 2)
% [row, column]= t_findArrayToMatrixIndex(6, 3) % should return (2, 3)
function [row, column] = findArrayToMatrixIndex(index, num_columns)
%(xmax, ymax)   
    % 1 2 3 
    % 4 5 6
           %(xmin, ymin)
    row = ceil(index/num_columns);
    column = rem(index, num_columns);
    if column==0
        column = num_columns;
    end
end