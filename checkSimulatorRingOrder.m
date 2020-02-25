function is_sorted = checkSimulatorRingOrder(LiDAR_ring_points)
    num_ring = size(LiDAR_ring_points, 2);
    for ring =1:num_ring
        z_axis(ring) = max(LiDAR_ring_points(ring).points.z);
    end
%     is_sorted = issorted(z_axis);
    [~, idx] = sort([z_axis(:)]);
    rings_order = linspace(1, num_ring, num_ring);
    is_sorted = isequal(idx', rings_order);
    
    if  is_sorted
        return
    else
        [rings_order' idx]
        error("Rings are mis-ordered in LiDAR simulator, please adjust noise level.")
    end
end