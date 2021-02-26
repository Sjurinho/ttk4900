truck_Pgt = out.location_gt;
truck_Thetagt = out.orientation_gt;

frontBumper_ptCloud = out.frontBumper_lidarPc.signals.values;
frontBumper_lidarPgt = out.frontBumper_lidarPgt.signals.values;
frontBumper_lidarOrientationgt = out.frontBumper_lidarOrientationgt.signals.values;
frontBumper_lidarRanges = out.frontBumper_lidarRanges.signals.values;
frontBumper_lidarTime = out.frontBumper_lidarPc.time;

rearBumper_ptCloud = out.rearBumper_lidarPc.signals.values;
rearBumper_lidarPgt = out.rearBumper_lidarPgt.signals.values;
rearBumper_lidarOrientationgt = out.rearBumper_lidarOrientationgt.signals.values;
rearBumper_lidarRanges = out.rearBumper_lidarRanges.signals.values;
rearBumper_lidarTime = out.rearBumper_lidarPc.time;

imu.accelerometer = out.accelerometer.signals.values;
imu.gyroscope     = out.gyroscope.signals.values;
imu.time = out.gyroscope.time;

frontBumperLidar = struct('ptCloud', frontBumper_ptCloud, 'lidarPgt', frontBumper_lidarPgt, ...
'lidarOrientationgt', frontBumper_lidarOrientationgt, 'lidarRanges', frontBumper_lidarRanges, ...
'time', frontBumper_lidarTime,'specs', frontBumper_lidarSpecs);
rearBumperLidar = struct('ptCloud', rearBumper_ptCloud, 'lidarPgt', rearBumper_lidarPgt, ...
'lidarOrientationgt', rearBumper_lidarOrientationgt, 'lidarRanges', rearBumper_lidarRanges, ...
'time', rearBumper_lidarTime,'specs', rearBumper_lidarSpecs);

descstring = 'This dataset is recorded using Unreal Engine and Simulink.';

sensorSpecs.frontBumper_lidar = frontBumper_lidarSpecs;
sensorSpecs.rearBumper_lidar = rearBumper_lidarSpecs;
sensorSpecs.imu = imuParams;

simple_tunnel_ds.frontBumperLidar = frontBumperLidar;
simple_tunnel_ds.rearBumperLidar = rearBumperLidar;
simple_tunnel_ds.Pgt = truck_Pgt;
simple_tunnel_ds.Thetagt = truck_Thetagt;
simple_tunnel_ds.imu = imu;
simple_tunnel_ds.sensorSpecs = sensorSpecs;
simple_tunnel_ds.Description = descstring;

save('datasets/SimpleTunnel_ds.mat', 'simple_tunnel_ds')