function [object_list, LiDAR_ring_points] = simulateCalibratedLiDAR(object_list, LiDAR_ring_points, LiDAR_opts, delta, opt_method)
    num_beam = LiDAR_opts.properties.beam;
    num_obj = length(object_list);
    
    %calibrate LiDAR ring points
%     for ring_num = 1:num_beam
%         LiDAR_ring_points(ring_num).homogenous_points = [LiDAR_ring_points(ring_num).points.x;
%                                                          LiDAR_ring_points(ring_num).points.y;
%                                                          LiDAR_ring_points(ring_num).points.z;
%                                                          ones(size(LiDAR_ring_points(ring_num).points.x))];
%         calibrated_points = delta(ring_num).Affine * LiDAR_ring_points(ring_num).homogenous_points;
%         LiDAR_ring_points(ring_num).calibrated_points.x = calibrated_points(1,:);
%         LiDAR_ring_points(ring_num).calibrated_points.y = calibrated_points(2,:);
%         LiDAR_ring_points(ring_num).calibrated_points.z = calibrated_points(3,:);
%     end
    
    %calibrate objects points

    if(opt_method == "Lie")
        for object = 1:num_obj
            object_list(object).calibrated_points_mat = [];
            for ring_num = 1:num_beam
                original_points = [object_list(object).ring_points(ring_num).x;
                                   object_list(object).ring_points(ring_num).y;
                                   object_list(object).ring_points(ring_num).z;
                                   ones(size(object_list(object).ring_points(ring_num).x))];
                if ~isempty(original_points)
                    % be careful with the indexing here. 0 or 1
                    calibrated_points = delta(ring_num+1).Affine*original_points;
                    object_list(object).calibrated_ring_points(ring_num).x = calibrated_points(1,:);
                    object_list(object).calibrated_ring_points(ring_num).y = calibrated_points(2,:);
                    object_list(object).calibrated_ring_points(ring_num).z = calibrated_points(3,:);
                    calibrated_points_XYZIR = [object_list(object).calibrated_ring_points(ring_num).x; ...
                                               object_list(object).calibrated_ring_points(ring_num).y; ...
                                               object_list(object).calibrated_ring_points(ring_num).z; ...
                                               object_list(object).ring_points(ring_num).I; ...
                                               object_list(object).ring_points(ring_num).R];
                    object_list(object).calibrated_points_mat = [object_list(object).calibrated_points_mat, calibrated_points_XYZIR ];

                end
            end
        end
    elseif (opt_method == "Spherical")
        for object = 1:num_obj
            object_list(object).calibrated_points_mat = [];
            for ring_num = 1:num_beam
                original_points = [object_list(object).ring_points(ring_num).x;
                                   object_list(object).ring_points(ring_num).y;
                                   object_list(object).ring_points(ring_num).z;
                                   ones(size(object_list(object).ring_points(ring_num).x))];
                if ~isempty(original_points)
                    % be careful with the indexing here. 0 or 1
                    spherical_points = Cartesian2Spherical(original_points);
                    object_list(object).calibrated_ring_points(ring_num).x = (spherical_points(1,:)+delta(ring_num+1).D).*sin(spherical_points(2,:)+delta(ring_num+1).theta).*cos(spherical_points(3,:)+delta(ring_num+1).phi);
                    object_list(object).calibrated_ring_points(ring_num).y = (spherical_points(1,:)+delta(ring_num+1).D).*sin(spherical_points(2,:)+delta(ring_num+1).theta).*sin(spherical_points(3,:)+delta(ring_num+1).phi);
                    object_list(object).calibrated_ring_points(ring_num).z = (spherical_points(1,:)+delta(ring_num+1).D).*cos(spherical_points(2,:)+delta(ring_num+1).theta);
                    calibrated_points_XYZIR = [object_list(object).calibrated_ring_points(ring_num).x; ...
                                               object_list(object).calibrated_ring_points(ring_num).y; ...
                                               object_list(object).calibrated_ring_points(ring_num).z; ...
                                               object_list(object).ring_points(ring_num).I; ...
                                               object_list(object).ring_points(ring_num).R];
                    object_list(object).calibrated_points_mat = [object_list(object).calibrated_points_mat, calibrated_points_XYZIR ];

                end
            end
        end
    end
end