function [object_list, color_list] = getScene26()    
    %% Description: 
%      a complex scene for validation. We place eighttargets at different locations with various angles
    

    %% Create object list
    num_obj = 3;    
    object_list(num_obj).object_vertices = struct();
    color_list = getColors(num_obj);
    
    %% Create objects
    objects1 = genShape("polygon", 3, 4);
%     objects2 = genShape("polygon", 3, 5);
    
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
    rpy = [0 30 20]; % in degree
    xyz = [0, -5, 0];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);

    rpy = [0 -20 25]; % in degree
    xyz = [-10, 10, 0];
    moved_obj2_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    rpy = [0 40 45]; % in degree
    xyz = [-10, -10, 0];
    moved_obj3_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
%     rpy = [0 0 35]; % in degree
%     xyz = [0, 5, 0];
%     moved_obj4_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
%     object_list(4).object_vertices = convertXYZmatrixToXYZstruct(moved_obj4_mat_h);
%     
%     
%     % Move Object2
%     object2_mat = convertXYZstructToXYZmatrix(objects2);
%     object2_mat_h = converToHomogeneousCoord(object2_mat);
% 
%     rpy = [20 0 15]; % in degree
%     xyz = [10, 10, 0.1];
%     moved_obj5_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
%     object_list(5).object_vertices = convertXYZmatrixToXYZstruct(moved_obj5_mat_h);
% 
%     rpy = [0 0 -25]; % in degree
%     xyz = [5, 0, 0.2];
%     moved_obj6_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
%     object_list(6).object_vertices = convertXYZmatrixToXYZstruct(moved_obj6_mat_h);
% 
%     rpy = [10 -10 5]; % in degree
%     xyz = [10, -10, 0.1];
%     moved_obj7_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
%     object_list(7).object_vertices = convertXYZmatrixToXYZstruct(moved_obj7_mat_h);
% 
%     rpy = [-25 30 -35]; % in degree
%     xyz = [-5, 0, 0.2];
%     moved_obj8_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
%     object_list(8).object_vertices = convertXYZmatrixToXYZstruct(moved_obj8_mat_h);
    
end