function noise_model = whiteNoiseRotatingHead(ring_num, LiDAR_opts)  
    
    % Ring number
    noise_model.type = 'additive';
    noise_model.ring = ring_num;
    noise = -0.1 + 0.2.*rand(3, 1);
    noise_model.x =  noise(1);
    noise_model.y =  noise(2);
    noise_model.z =  noise(3);

end
