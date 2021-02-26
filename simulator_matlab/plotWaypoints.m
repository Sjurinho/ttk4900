function plotWaypoints(scenario,refPath)
%PLOTWAYPOINTS Summary of this function goes here
%   Detailed explanation goes here
refPosesX = refPath(:, 1);
refPosesY = refPath(:, 2);
refPosesT = refPath(:, 3);
sceneImage = imread(strcat(scenario,'.png'));
imageSize = size(sceneImage);
xlims = [-1530, 27750]; %cm
ylims = [-1750, 1450]; %cm

sceneRef = imref2d(imageSize,xlims/100,ylims/100);
hScene = figure;
hIm = imshow(sceneImage, sceneRef);

set(gca, 'YDir', 'reverse', 'Visible', 'on')

xlabel('X (m)')
ylabel('Y (m)')

hold on;
scatter(refPosesX, refPosesY, [], 'filled', 'DisplayName', ...
    'Reference Path');
legend
hold off;
end

