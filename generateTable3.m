format short g
scenes = [8,9,10,11,12,6,14,7];
noise_model = [2,3,6];
methods = ["\BaseLine1\", "\BaseLine3\","\Lie\"];

for model = 3:3
    summary = [];
    for i = 1: size(scenes,2)
        scene = scenes(i);
        block1 = [];
        block2 = [];
        col = [];
        for method = 1:size(methods,2)
            load_path = ".\results\noiseModel" + num2str(noise_model(model))+ "\Validation13"+ "\scene" + num2str(scene)+ methods(method) + "\data.mat";
            load(load_path)
            block1 = [block1; results(1:16).mean_calibrated; results(1:16).mean_percentage];
            block2 = [block2; results(17:32).mean_calibrated; results(17:32).mean_percentage];
            col = [col; mean(abs([results(:).mean_calibrated]))];
        end
        block1 = [results(1:16).mean_original; block1]
        block2 = [results(17:32).mean_original; block2]
        summary = [summary, col];
    end
end
summary