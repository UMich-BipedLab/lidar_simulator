function pick_color = getColors(num_colors)
    color_list = ["red", "green", "blue", "magenta", "brown", "cyan", ...
                   "darkgray", "gray", "lightgray", "lime", "yellow", ...
                  "olive", "orange", "pink", "purple", "teal", "violet"];
    num_color_list = length(color_list);
    if num_colors <= num_color_list
        pick_color = color_list(1:num_colors);
    else
        pick_color = color_list;
        remainder = mod(num_colors, num_color_list);
        quotient = floor(num_colors ./ num_color_list);
        for i=1:(quotient-1) % copy once
            pick_color = [pick_color, color_list];
        end
        pick_color = [color_list, color_list(1:remainder)];
    end
end