function ring_noise_model = genLiDARNoiseModel(segmentation_type, LiDAR_opts)
    if LiDAR_opts.mechanism.types(LiDAR_opts.mechanism.type) == "rotating-head"
        ring_noise_model = genRotatingHeadLiDARNoiseModel(segmentation_type, LiDAR_opts);
        
    elseif LiDAR_opts.mechanism.types(LiDAR_opts.mechanism.type) == "solid-state"
        ring_noise_model = genSolidStateLiDARNoiseModel(segmentation_type, LiDAR_opts); 
    else
        error("No such LiDAR type: %s", ...
              LiDAR_opts.mechanism.types(LiDAR_opts.mechanism.type))
    end
end