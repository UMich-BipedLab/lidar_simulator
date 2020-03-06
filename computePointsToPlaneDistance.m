function distance = computePointsToPlaneDistance(data, plane, num_beams)
%     num_targets = size(plane, 2);
%     for n = 1:num_beams
%         X = [];
%         for t = 1:num_targets
%             point = data{t}(n).points;
%             num_points = size(point, 2); 
%             plane_centroids = plane(t).centroid;
%             normals = plane(t).unit_normals;
%             if num_points > 0
%                 for k = 1:num_points
%                     diff = [point(1:3, k) - plane_centroids];
%                     X = [X abs((dot(normals, diff(1:3,:))))];
%                 end
%             end
%         end
%     end
%     distance
    num_targets = size(plane, 2);
    distance.ring(num_beams) = struct();
    
    for n = 1:num_beams
        X = [];
        points_array = [];
        sum_points = 0;
        for t = 1:num_targets
            point = data{t}(n).points;
            num_points = size(point, 2); 
            if num_points > 0
                plane_centroids = repmat(plane(t).centroid, [1, num_points]);
                points_array = [points_array point(1:3,:)];
                diff = [point(1:3,:) - plane_centroids];
                normals = repmat(plane(t).unit_normals, [1, num_points]);
                X = [X abs((dot(normals, diff(1:3,:))))];
            end
            sum_points = sum_points + num_points;
        end
        distance.ring(n).ring = n;
        distance.ring(n).num_points = sum_points;
        if sum_points > 0
            distance.ring(n).mean = mean(X,2);
            ring_centroid = mean(points_array, 2);
            distance.ring(n).ring_centroid = ring_centroid(1:3)';
            distance.ring(n).z_axis = ring_centroid(3);
            distance.ring(n).std = std(X);
        else
            distance.ring(n).ring_centroid = [0, 0, 0];
            distance.ring(n).z_axis = 0;
            distance.ring(n).mean = 0;
            distance.ring(n).std = 0;
        end
        distance.ring(n).mean_in_mm = distance.ring(n).mean * 1e3;
        distance.ring(n).std_in_mm = distance.ring(n).std * 1e3;
    end
    distance.mean = mean(abs([distance.ring(:).mean]));
end