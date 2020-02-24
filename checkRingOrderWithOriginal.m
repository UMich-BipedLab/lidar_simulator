function [ring_ordered, ring_misorder_list] = checkRingOrderWithOriginal(data_split_with_ring_original, data_split_with_ring, num_targets, num_ring)
    ring_order(num_ring) = struct();
    ring_order_original(num_ring) = struct();
    
    for ring =1:num_ring
        ring_points = [];
        ring_points_org = [];
        for i =1:num_targets  
            if (isempty(data_split_with_ring_original{i}(ring).points))
                continue;
            else
                ring_points_org = [ring_points_org, data_split_with_ring_original{i}(ring).points];
            end
            
            if (isempty(data_split_with_ring{i}(ring).points))
                continue;
            else
                ring_points = [ring_points, data_split_with_ring{i}(ring).points];
            end
        end
        
        ring_order(ring).ring = ring;
        if ~isempty(ring_points)
            ring_order(ring).ring_mean = mean(ring_points(1:3, :), 2)';
            ring_order(ring).z_axis = ring_order(ring).ring_mean(3);
        else
            ring_order(ring).ring_mean = [0 0 0];
            ring_order(ring).z_axis = 0;
        end
        
        ring_order_original(ring).ring = ring;
        if ~isempty(ring_points_org)
            ring_order_original(ring).ring_mean = mean(ring_points_org(1:3, :), 2)';
            ring_order_original(ring).z_axis = ring_order_original(ring).ring_mean(3);
        else
            ring_order_original(ring).ring_mean = [0 0 0];
            ring_order_original(ring).z_axis = 0;
        end
    end
    
    [~, idx] = sort([ring_order(:).z_axis]);
    ordered_ring = ring_order(idx);
    
    [~, idx] = sort([ring_order_original(:).z_axis]);
    ordered_ring_original = ring_order_original(idx);
    
    
    diff_vec = find([ordered_ring_original.ring] ~= [ordered_ring.ring]);
    
    if  isempty(diff_vec)
        disp("Rings are ordered!")
        ring_misorder_list = ring_order;
    else
        warning("Rings are mis-ordered.....")
        [ordered_ring_original.ring; ordered_ring.ring]'
    end
end