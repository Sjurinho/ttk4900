
if exist('scenario', 'var') == 0
    scenario = 'straightTunnel_long';
end
    
sceneImage = imread(strcat(scenario, '.png'));
imageSize = size(sceneImage);
%xlims = [-1530, 27750]; %cm
%ylims = [-1750, 1450]; %cm
xlims = [-10000.0, 35000.0]; %cm
ylims = [-3000.0, 2000.0]; %cm

sceneRef = imref2d(imageSize,xlims/100,ylims/100);

hFig = helperSelectSceneWaypoints(sceneImage, sceneRef);
set(gca, 'Ydir', 'reverse')
uiwait(hFig);
uiwait(hFig);

refPath = refPoses{1,1};

save(strcat('refpath_nonsmooth_', scenario, '.mat'), 'refPath');
% 
% [smoothRefPoses,~,cumLengths] = smoothPathSpline(refPath, refDirections, numSmoothPoses);
% 
% timeVector = normalize(cumLengths, 'range', [0, T]);
% 
% RefPath.refPosesX = [timeVector, smoothRefPoses(:, 1)];
% RefPath.refPosesY = [timeVector, smoothRefPoses(:, 2)];
% 
% for i = 1:length(smoothRefPoses(:,3))
%     smoothRefPoses(i, 3) = ssa(smoothRefPoses(i, 3), 'deg');
% end
% RefPath.refPosesT = [timeVector, smoothRefPoses(:, 3)];
% 
% 
% save('RefPath.mat', 'RefPath')