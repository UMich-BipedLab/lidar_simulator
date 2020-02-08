% testing code
% clc, clear, close all
% handle = t_createFigHandleWithNumber(2, 5,"test");
% plot(handle(1),[1 2 3],[2 4 6]);
% plot(handle(2),[10 20 30],[20 40 60]);
% set(get(handle(1),'parent'),'visible','on');% show the current axes
% set(get(handle(2),'parent'),'visible','on');% show the current axes


function fig_handle = createFigHandleWithNumber(num_handles, start_number, name)
%     fig_handle = zeros(1,num_handles);
%     fig_handle = cell(1, num_handles);
    start_number = max(1, start_number);
    fig_handle = [];
    for i = start_number : start_number+num_handles-1
        if ishandle(i)
            fig = figure(i);
            clf(fig,'reset');
            set(fig, 'Name', name + "-" + num2str(i-start_number+1), 'Visible', 'on');
        else
            fig = figure(i);
            set(fig, 'Name', name + "-" + num2str(i-start_number+1), 'Visible', 'off');
        end
        axis(fig.CurrentAxes, 'equal')
        fig_handle = [fig_handle; axes('parent', fig)];
    end
end