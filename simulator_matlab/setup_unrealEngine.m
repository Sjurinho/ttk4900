%% STEP1
% Specify the location of the support package project files and a local folder destination
% Note:  Only one path is supported. Select latest download path.
dest_root = "C:\Local";
src_root = fullfile(matlabshared.supportpkg.getSupportPackageRoot, ...
    "toolbox", "shared", "sim3dprojects", "automotive");

%% STEP2
% Specify the location of the Unreal Engine installation.
ueInstFolder = "C:\Program Files\Epic Games\UE_4.23";

%% STEP3
% Copy the MathWorksSimulation plugin to the Unreal Engine plugin folder.
mwPluginName = "MathWorksSimulation";
mwPluginFolder = fullfile(src_root, "PluginResources", "UE423"); % choose UE version
uePluginFolder = fullfile(ueInstFolder, "Engine", "Plugins");
uePluginDst = fullfile(uePluginFolder, "Marketplace", "MathWorks");

cd(uePluginFolder)
foundPlugins = dir("**/" + mwPluginName + ".uplugin");

if ~isempty(foundPlugins)
    numPlugins = size(foundPlugins, 1);
    msg2 = cell(1, numPlugins);
    pluginCell = struct2cell(foundPlugins);

    msg1 = "Plugin(s) already exist here:" + newline + newline;
    for n = 1:numPlugins
        msg2{n} = "    " + pluginCell{2,n} + newline;
    end
    msg3 = newline + "Please remove plugin folder(s) and try again.";
    msg  = msg1 + msg2 + msg3;
    warning(msg);
else
    copyfile(mwPluginFolder, uePluginDst);
    disp("Successfully copied MathWorksSimulation plugin to UE4 engine plugins!")
end

%% STEP4
% Copy the support package folder that contains the AutoVrtlEnv.uproject
% files to the local folder destination.
projFolderName = "AutoVrtlEnv";
projSrcFolder = fullfile(src_root, projFolderName);
projDstFolder = fullfile(dest_root, projFolderName);
if ~exist(projDstFolder, "dir")
    copyfile(projSrcFolder, projDstFolder);
end