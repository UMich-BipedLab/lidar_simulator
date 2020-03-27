%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

clc, clear

% Tag Family Setting
tag.family = 4; % LiDARTag family
tag.black_border = 1; 
tag.white_border = 1;
tag.grid_size = 0.5; % size of each grid
tag.size = tag.grid_size * (tag.family + 2*tag.black_border + 2*tag.white_border);
img = imread('tag16_05_00001.png'); % Load LiDARTag

% Plot Setting
plotting.grid_size = 0.5 * tag.grid_size;
plotting.height = 0 * tag.grid_size;
plotting.x_dim = (tag.size + tag.grid_size) / 2;
plotting.y_dim = (tag.size + tag.grid_size) / 2;
plotting.img_x = tag.size/2;
plotting.img_y = tag.size/2;
plot_x = -plotting.x_dim: plotting.grid_size : plotting.x_dim;
plot_y = -plotting.x_dim: plotting.grid_size : plotting.x_dim;
[plot_x, plot_y] = meshgrid(plot_x, plot_y);
plot = [plot_x(:) plot_y(:)];


% Show Image in 3D
figure(1)
img_x = -plotting.img_x: plotting.grid_size : plotting.img_x;
img_y = -plotting.img_y: plotting.grid_size : plotting.img_y;
image(img_x(:),img_y(:),img, 'AlphaData', 0.5); 
hold on;

% Construct Gaussian Distribution
y = 0;
sigma = [tag.grid_size/10 0; 0 tag.grid_size/10];
for i = 1:size(img,1)
    for j = 1:size(img,2)
        peak = img(i,j,1);
        if peak==255
            peak = 1;
        elseif peak == 0
            peak = -1;
        end
        mu = [(tag.grid_size*j-plotting.x_dim) (tag.grid_size*i-plotting.y_dim)];
        disp = peak .* mvnpdf(plot,mu,sigma);
        y = y + disp;       
    end
end
y = reshape(y,length(plot_y),length(plot_x));
y = y + plotting.height;
surf(plot_x, plot_y, y, 'AlphaData', 0.1,'FaceAlpha', 0.5)



% Figure Setting
% view(-30,40)
view(0,45)
view(90,70)
view(180,-90)
% caxis([min(y(:))-0.5*range(y(:)),max(y(:))])
% axis([-3 3 -3 3 0 0.4])
% xlabel('x')
% ylabel('y')
% zlabel('Probability Density')

axis equal
hold off
axis off
fig = gcf;
set(fig, 'Color', 'None')
% h1 = axes;
% set(gca,'Ydir','reverse')
% scatter3(0,0,0)
% scatter3(0,1,0)


% figure(2)
% imshow(img, 'InitialMagnification', 1000);

%%