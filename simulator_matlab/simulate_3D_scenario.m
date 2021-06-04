% data = load('RefPath.mat');
% 
% refPosesX = data.RefPath.refPosesX;
% refPosesY = data.RefPath.refPosesY;
% refPosesT = data.RefPath.refPosesT;

scenario = 'straightTunnel_long';
wp_fn = strcat('refpath_nonsmooth_', scenario, '.mat');
inputstr = "Would you like to create a new scenario?";

if exist(wp_fn, 'file') > 0
    load(wp_fn);
    plotWaypoints(scenario, refPath);
    newScenario = input("Would you like to create a new scenario?(1 for yes, 0 for no)");
    close all;
else
    disp("No scenario exists, please specify a new one");
    newScenario=1;
end

if newScenario==1
    select_waypoints;
    load(wp_fn);
end

numPoses = size(refPath, 1);

refDirections  = ones(numPoses,1);   % Forward-only motion
numSmoothPoses = 50 * numPoses;      % Increase this to increase the number of returned poses
refVelocities  = 1.5 * ones(numPoses,1);

setImuParameters;
setLidarParameters;

T = 1000;
plotWaypoints(scenario, refPath);

modelName = 'sim3d';
open_system(modelName);
mh=get_param(modelName,'handle'); % Returns model handle
set_param(mh,'StopFcn','save_3DSim_data');
snapnow;