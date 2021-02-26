
imuParams.acc_resolution = 0.00059875;
imuParams.acc_constantBias = 0;%0.49; %[0.49, 0.49, 0.49];
imuParams.acc_velocity_randomWalk = 0.003924; %[0.003924, 0.003924, 0.003924];
imuParams.acc_biasInstability = 0;
imuParams.acc_acceleration_randomWalk = 0.0015398; %[0.0015398,0.0015398,0.0015398];

imuParams.gyro_resolution = 0.00013323;
imuParams.gyro_constantBias = 0;%[0.3491,0.5,0];
imuParams.gyro_accelerationBias = 0.00017809;
imuParams.gyro_angle_randomWalk = 8.7266e-05; %[8.7266e-05,8.7266e-05,8.7266e-05];
imuParams.gyro_biasInstability = 0;
imuParams.gyro_rate_randomWalk = 3.24e-05; %[3.24e-05, 3.24e-05, 3.24e-05];;

IMUALIGNMENT = eul2rotm([-pi, 0, 0]);