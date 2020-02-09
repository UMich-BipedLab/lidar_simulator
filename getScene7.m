function [object_list, color_list] = getScene7()    
    %% Description: 
    % 8 big, 8 medium, 8 small (square)
    % spread with different angles around the lidar
    

    %% Create object list
    num_obj = 24;    
    object_list(num_obj).object_vertices = struct();
    color_list = getColors(num_obj);
    
    %% Create objects
    objects1 = genShape("polygon", 0.5, 4);
    objects2 = genShape("polygon", 1.2, 4);
    objects3 = genShape("polygon", 1.8, 4);
    
    % Plot original polygon (2D)
    % pgon = polyshape(objects1.y, objects1.z);
    % plot(fig_handle(1), pgon)
    % viewCurrentPlot(fig_handle(1), "2D")

    % Plot original polygon (3D)
    % plotConnectedVerticesStructure(fig_handle(2), vertices, 'b')

    %% move away the polygon
    disp("--- Moving obstacles...")
    % covert format
    object1_mat = convertXYZstructToXYZmatrix(objects1);
    object1_mat_h = converToHomogeneousCoord(object1_mat);
    object2_mat = convertXYZstructToXYZmatrix(objects2);
    object2_mat_h = converToHomogeneousCoord(object2_mat);
    object3_mat = convertXYZstructToXYZmatrix(objects3);
    object3_mat_h = converToHomogeneousCoord(object3_mat);
    
    %% small targets
    % Object1
    rpy = [10 1 25]; % in degree
    xyz = [2.5, 3, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
        % Object1
    rpy = [10 1 25]; % in degree
    xyz = [2.5, 3, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
        % Object1
    rpy = [10 1 25]; % in degree
    xyz = [2.5, 3, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
        % Object1
    rpy = [10 1 25]; % in degree
    xyz = [2.5, 3, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    %% medium targets

    % Move Object2
    rpy = [5 8 15]; % in degree
    xyz = [5, 2, 0.1];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);

    % Move Object3
    rpy = [5 10 -25]; % in degree
    xyz = [7, -3, 0.2];
    moved_obj3_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);

    
    %% large targets
    % Move Object4
    rpy = [5 5 0]; % in degree
    xyz = [10, 0, 0];
    moved_obj4_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(4).object_vertices = convertXYZmatrixToXYZstruct(moved_obj4_mat_h);
end