function pick_color = getColors(num_colors)
%     color_list = ["red", "green", "blue", "magenta", "cyan", ...
%                   "yellow"];
    color_list = {[1 0 0], [0 1 0], [0 0 1], [1 0 1], [0 1 1], [1 1 0]};
    num_color_list = length(color_list);
    if num_colors <= num_color_list
        pick_color = color_list(1:num_colors);
    else
%         pick_color = color_list;
%         remainder = mod(num_colors, num_color_list);
%         quotient = floor(num_colors ./ num_color_list);
%         
%         for i=1:(quotient-1) % copy once
%             pick_color = [pick_color, color_list];
%         end
%         pick_color = [color_list, color_list(1:remainder)];
        pick_color = color_list;
        remainder = num_colors - num_color_list;
        for i=1:remainder 
            pick_color = [pick_color, {rand(1,3)}];
        end
    end
end