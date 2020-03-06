
function new_points = measureSolidStateEndPoints(points, lidar_pose, desire_rotation, range)
    % desire_rotation in [roll pitch yaw] in degree
    if ~isvector(desire_rotation)
        error("rotation should be in [r p y] format")
    end
    
    [~, points] = checkHomogeneousCoord(points);

%     mean(points')
    
    x_G = points;
    H_GL = [lidar_pose(1:3,1:3)', -lidar_pose(1:3,1:3)*lidar_pose(1:3, 4);
           0 0 0 1];
    T_L =  constructHByRPYXYZ(desire_rotation, [range, 0 0]);
    H_LL_prime = [T_L(1:3,1:3), -T_L(1:3,1:3)*T_L(1:3, 4);
                  0 0 0 1];
    H_LG = inv(H_GL*H_LL_prime);
    new_points = H_LG * T_L * H_GL * x_G;
    
%     num_points = size(points,2);
%     new_points = ones(4, size(points, 2));
%     
%     for i = 1:num_points
%         x_G = points;
%         H_GL = [points_t.diode(:).T(1:3,1:3)', -points_t.diode(:).T(1:3,1:3)*points_t.diode(:).T(1:3,4);
%                0 0 0 1];
%         T_L =  constructHByRPYXYZ(desire_rotation, [range, 0 0]);
%         H_LG = inv(H_GL);
%         new_points(:, i) = H_LG * T_L * H_GL * x_G;
%     end
%     p = [0 0 0 1]
%     T = lidar_pose
%     (lidar_pose * p')'
%     H_GL
%     T_L
%     mean((H_GL * x_G)')
%     mean((T_L*H_GL * x_G)')
%     mean((H_LG*T_L*H_GL * x_G)')
%     vec = lidar_pose(1:3,1:3) * [1 0 0]';
%     movement = vec' .* [range, range, range];
%     H = constructHByRPYXYZ(desire_rotation, movement);
%     new_points = H * points;
end