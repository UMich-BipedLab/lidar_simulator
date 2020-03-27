function [object_list, color_list] = getScene24()    
    %% Description:  
%      two  targets  are  placed  at  equal  distance  andopposite  sides  
%      along  thex-axis  of  the  LiDAR,  parallel to one another and 
%      to the z-axis (N1,N2are not linearlyindependent)
    
    

    %% Create object list
    num_obj = 3;    
    object_list(num_obj).object_vertices = struct();
    color_list = getColors(num_obj);
    
    %% Create objects
    objects1 = genShape("polygon", 3, 4);
    
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

    % Move Object2
    rpy = [0 0 90]; % in degree
    xyz = [0, 6, 0];
    moved_obj2_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);

    rpy = [0 -45 0]; % in degree
    xyz = [-8, 0, 0];
    moved_obj3_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
end