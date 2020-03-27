function [object_list, color_list] = getScene30()    
    %% Description:  
%      two  targets  are  placed  at  equal  distance  andopposite  sides  
%      along  thex-axis  of  the  LiDAR,  parallel to one another and 
%      to the z-axis (N1,N2are not linearlyindependent)
    
    

    %% Create object list
    num_obj = 7;    
    object_list(num_obj).object_vertices = struct();
    color_list = getColors(num_obj);
    
    %% Create objects
    target_size = 6;
    objects1 = genShape("polygon", target_size, 4);
    
    % Plot original polygon (2D)
    % pgon = polyshape(objects1.y, objects1.z);
    % plot(fig_handle(1), pgon)
    % viewCurrentPlot(fig_handle(1), "2D")

    % Plot original polygon (3D)
    % plotConnectedVerticesStructure(fig_handle(2), vertices, 'b')

    %% move away the polygon
    disp("--- Moving obstacles...")
    
    % Move Object1
    object1_mat = convertXYZstructToXYZmatrix(objects1);
    object1_mat_h = converToHomogeneousCoord(object1_mat);
    
    rpy = [0 45 0]; % in degree
    xyz = [6, 0, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    [object_list(1:num_obj).size] = deal(target_size);

    % Move Object2
    rpy = [0 0 90]; % in degree
    xyz = [0, 6, 0];
    moved_obj2_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);

    rpy = [0 -45 0]; % in degree
    xyz = [-8, 0, 0];
    moved_obj3_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    rpy = [30  0 75]; % in degree
    xyz = [0, -10, 0];
    moved_obj4_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(4).object_vertices = convertXYZmatrixToXYZstruct(moved_obj4_mat_h);
    
    rpy = [0 20 60]; % in degree
    xyz = [8, 8, 0];
    moved_obj5_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(5).object_vertices = convertXYZmatrixToXYZstruct(moved_obj5_mat_h);

    rpy = [0 -15 30]; % in degree
    xyz = [-8, -8, 0];
    moved_obj6_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(6).object_vertices = convertXYZmatrixToXYZstruct(moved_obj6_mat_h);
    
    rpy = [20  0 30]; % in degree
    xyz = [10, -10, 0];
    moved_obj7_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(7).object_vertices = convertXYZmatrixToXYZstruct(moved_obj7_mat_h);
end