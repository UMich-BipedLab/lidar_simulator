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
                    calibrated_points = delta(ring_num).Affine *original_points;
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
    elseif (opt_method == "BaseLine1")
        for object = 1:num_obj
            object_list(object).calibrated_points_mat = [];
            for ring_num = 1:num_beam
                original_points = [object_list(object).ring_points(ring_num).x;
                                   object_list(object).ring_points(ring_num).y;
                                   object_list(object).ring_points(ring_num).z;
                                   ones(size(object_list(object).ring_points(ring_num).x))];
                if ~isempty(original_points)
                    spherical_points = Cartesian2Spherical(original_points);
                    object_list(object).calibrated_ring_points(ring_num).x = (spherical_points(1,:)+delta(ring_num).D).*sin(spherical_points(2,:)+delta(ring_num).theta).*cos(spherical_points(3,:)+delta(ring_num).phi);
                    object_list(object).calibrated_ring_points(ring_num).y = (spherical_points(1,:)+delta(ring_num).D).*sin(spherical_points(2,:)+delta(ring_num).theta).*sin(spherical_points(3,:)+delta(ring_num).phi);
                    object_list(object).calibrated_ring_points(ring_num).z = (spherical_points(1,:)+delta(ring_num).D).*cos(spherical_points(2,:)+delta(ring_num).theta);
                    calibrated_points_XYZIR = [object_list(object).calibrated_ring_points(ring_num).x; ...
                                               object_list(object).calibrated_ring_points(ring_num).y; ...
                                               object_list(object).calibrated_ring_points(ring_num).z; ...
                                               object_list(object).ring_points(ring_num).I; ...
                                               object_list(object).ring_points(ring_num).R];
                    object_list(object).calibrated_points_mat = [object_list(object).calibrated_points_mat, calibrated_points_XYZIR ];

                end
            end
        end
    elseif (opt_method == "BaseLine2")
        for object = 1:num_obj
            object_list(object).calibrated_points_mat = [];
            for ring_num = 1:num_beam
                original_points = [object_list(object).ring_points(ring_num).x;
                                   object_list(object).ring_points(ring_num).y;
                                   object_list(object).ring_points(ring_num).z;
                                   ones(size(object_list(object).ring_points(ring_num).x))];
                %Note:the ring number in validation is not one to one
                %corresponds to the delta. We need to consider several
                %cases.
                if ~isempty(original_points)
                    %No calibration parameter, maintain the data
                    if(delta(ring_num).D_s ==1 && delta(ring_num).D == 0 && delta(ring_num).A_c == 0 && delta(ring_num).opt_total_cost == 0)
                        object_list(object).calibrated_ring_points(ring_num).x = object_list(object).ring_points(ring_num).x;
                        object_list(object).calibrated_ring_points(ring_num).y = object_list(object).ring_points(ring_num).y;
                        object_list(object).calibrated_ring_points(ring_num).z = object_list(object).ring_points(ring_num).z;
                    else %Calibrate the points accordingly
                        spherical_points = Cartesian2Spherical(original_points);
                        dxy = (spherical_points(1,:)*delta(ring_num).D_s + delta(ring_num).D) * delta(ring_num).S_vc -delta(ring_num).C_voc;
                        object_list(object).calibrated_ring_points(ring_num).x = dxy.*cos(spherical_points(3,:)- delta(ring_num).A_c)- delta(ring_num).H_oc *sin(spherical_points(3,:)- delta(ring_num).A_c);
                        object_list(object).calibrated_ring_points(ring_num).y = dxy.*sin(spherical_points(3,:)- delta(ring_num).A_c)+ delta(ring_num).H_oc *cos(spherical_points(3,:)- delta(ring_num).A_c);
                        object_list(object).calibrated_ring_points(ring_num).z = (spherical_points(1,:)+ delta(ring_num).D)*delta(ring_num).C_vc + delta(ring_num).S_voc;                    
                    end
                    calibrated_points_XYZIR = [object_list(object).calibrated_ring_points(ring_num).x; ...
                                               object_list(object).calibrated_ring_points(ring_num).y; ...
                                               object_list(object).calibrated_ring_points(ring_num).z; ...
                                               object_list(object).ring_points(ring_num).I; ...
                                               object_list(object).ring_points(ring_num).R];
                    object_list(object).calibrated_points_mat = [object_list(object).calibrated_points_mat, calibrated_points_XYZIR ];
                end
            end
        end
    elseif (opt_method == "BaseLine3")
        for object = 1:num_obj
            object_list(object).calibrated_points_mat = [];
            for ring_num = 1:num_beam
                original_points = [object_list(object).ring_points(ring_num).x;
                                   object_list(object).ring_points(ring_num).y;
                                   object_list(object).ring_points(ring_num).z;
                                   ones(size(object_list(object).ring_points(ring_num).x))];
                %Note:the ring number in validation is not one to one
                %corresponds to the delta. We need to consider several
                %cases.
                if ~isempty(original_points)
                    %No calibration parameter, maintain the data
                    if(delta(ring_num).D_s ==1 && delta(ring_num).D == 0 && delta(ring_num).A_c == 0 && delta(ring_num).opt_total_cost == 0)
                        object_list(object).calibrated_ring_points(ring_num).x = object_list(object).ring_points(ring_num).x;
                        object_list(object).calibrated_ring_points(ring_num).y = object_list(object).ring_points(ring_num).y;
                        object_list(object).calibrated_ring_points(ring_num).z = object_list(object).ring_points(ring_num).z;
                    else %Calibrate the points accordingly
                        spherical_points = Cartesian2Spherical(original_points);
                        dxy = (spherical_points(1,:)*delta(ring_num).D_s + delta(ring_num).D) .* sin(delta(ring_num).V_c);
                        object_list(object).calibrated_ring_points(ring_num).x = dxy.*cos(spherical_points(3,:)- delta(ring_num).A_c)- delta(ring_num).H_oc *sin(spherical_points(3,:)- delta(ring_num).A_c);
                        object_list(object).calibrated_ring_points(ring_num).y = dxy.*sin(spherical_points(3,:)- delta(ring_num).A_c)+ delta(ring_num).H_oc *cos(spherical_points(3,:)- delta(ring_num).A_c);
                        object_list(object).calibrated_ring_points(ring_num).z = (spherical_points(1,:)*delta(ring_num).D_s + delta(ring_num).D).* cos(delta(ring_num).V_c) + delta(ring_num).V_oc;                    
                    end
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