function rand_num = genRandomNumber(range_min, range_max, seed, N)
    % return (N,1) random numbers in between range_minand range_max 
    if exist('seed', 'var')
        rng(seed)
    else
        rng(rand())
    end
    if ~exist("N", "var")
        N = 1;
    end
    rand_num = range_min + (range_max-range_min) .* rand(N, 1);
end
% 
% 
%     range_max =  0.005;
%     range_max =  0.1;
%     range_min = -range_max;
%     noise_model.range_noise = genRandomNumber(range_min, range_max, ring_num); 