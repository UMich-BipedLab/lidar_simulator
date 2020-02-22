function [object_list, color_list] = getScene7()    
    %% Description: 
    % 8 big, 8 medium, 8 small (square) well distributed in different positions and orentations 
    %slightly constrained in the elevation angle.(slightly tilted lower and
    %upper bowel). Rings less tend to be misordered.
    

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
    xyz = [2.5, 0, 0.2];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(1).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 -25]; % in degree
    xyz = [-2.5, 0, -0.2];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(2).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 95]; % in degree
    xyz = [0, 2.5, -0.3];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(3).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 -95]; % in degree
    xyz = [0, -2.5, 0.3];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(4).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    % Object1
    rpy = [10 1 40]; % in degree
    xyz = [2.5, 2.5, -0.1];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(5).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 -40]; % in degree
    xyz = [2.5, -2.5, 0.4];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(6).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 -50]; % in degree
    xyz = [-2.5, 2.5, -0.3];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(7).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    % Object1
    rpy = [10 1 50]; % in degree
    xyz = [-2.5, -2.5, 0.2];
    moved_obj1_mat_h = moveByRPYXYZ(object1_mat_h, rpy, xyz);
    object_list(8).object_vertices = convertXYZmatrixToXYZstruct(moved_obj1_mat_h);
    
    
    %% medium targets

    % Move Object2
    % Object1
    rpy = [10 1 25]; % in degree
    xyz = [6.5, 0, 0.2];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(9).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 -25]; % in degree
    xyz = [-5.5, 0.5, 0.4];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(10).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 95]; % in degree
    xyz = [-0.5, 5.5, 0.2];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(11).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 -95]; % in degree
    xyz = [0, -6, 0.3];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(12).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    % Object2
    rpy = [10 1 40]; % in degree
    xyz = [5.5, 6.5, -0.2];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(13).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 -40]; % in degree
    xyz = [6, -5.5, 0.1];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(14).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 -50]; % in degree
    xyz = [-6, 5.3, -0.3];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(15).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);
    
    % Object2
    rpy = [10 1 50]; % in degree
    xyz = [-5.5, -5.2, 0.1];
    moved_obj2_mat_h = moveByRPYXYZ(object2_mat_h, rpy, xyz);
    object_list(16).object_vertices = convertXYZmatrixToXYZstruct(moved_obj2_mat_h);

    
    %% large targets
    % Move Object3
    % Object3
    rpy = [10 1 25]; % in degree
    xyz = [15.5, 3, 0.2];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(17).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 -25]; % in degree
    xyz = [-15.5, -2, 0.4];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(18).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 95]; % in degree
    xyz = [-5.5, 15.5, 0.2];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(19).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 -95]; % in degree
    xyz = [1, -16, 0.3];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(20).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    % Object3
    rpy = [10 1 40]; % in degree
    xyz = [14.5, 16.5, -0.2];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(21).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 -40]; % in degree
    xyz = [17, -15.5, 0.1];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(22).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 -50]; % in degree
    xyz = [-14, 18.3, -0.3];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(23).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);
    
    % Object3
    rpy = [10 1 50]; % in degree
    xyz = [-15.5, -15.2, 0.1];
    moved_obj3_mat_h = moveByRPYXYZ(object3_mat_h, rpy, xyz);
    object_list(24).object_vertices = convertXYZmatrixToXYZstruct(moved_obj3_mat_h);

end