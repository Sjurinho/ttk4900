function gnss_struct = addGNSS(dataset, tunnel_start, tunnel_end, period_gps)
pgt_before_tunnel = dataset.Pgt.signals.values(:,:,dataset.Pgt.signals.values(:,1,:) < tunnel_start);
pgt_after_tunnel = dataset.Pgt.signals.values(:,:,dataset.Pgt.signals.values(:,1,:) > tunnel_end);

times_before_tunnel = dataset.Pgt.time(dataset.Pgt.signals.values(:,1,:) < tunnel_start);
times_after_tunnel = dataset.Pgt.time(dataset.Pgt.signals.values(:,1,:) > tunnel_end);

prev_measurement = times_before_tunnel(1);
measurements_before_tunnel = zeros(size(times_before_tunnel));
measurements_before_tunnel(1) = 1;
for i=1:length(times_before_tunnel)
    if times_before_tunnel(i) > (prev_measurement + period_gps)
        measurements_before_tunnel(i) = 1;
        prev_measurement = times_before_tunnel(i);
    end
end
prev_measurement = times_after_tunnel(1);
measurements_after_tunnel = zeros(size(times_after_tunnel));
measurements_after_tunnel(1) = 1;
for i=1:length(times_after_tunnel)
    if times_after_tunnel(i) > (prev_measurement + period_gps)
        measurements_after_tunnel(i) = 1;
        prev_measurement = times_after_tunnel(i);
    end
end

gnss_measurement_base = squeeze(cat(3, pgt_before_tunnel(:,:,measurements_before_tunnel == 1), pgt_after_tunnel(:,:,measurements_after_tunnel==1)));
gnss_measurement_times = cat(1, times_before_tunnel(measurements_before_tunnel==1), times_after_tunnel(measurements_after_tunnel==1));
perturbation = mvnrnd([0,0,0], [0.1, 0.1, 0.2], length(gnss_measurement_base));

gnss.measurements = gnss_measurement_base' + perturbation;
gnss.times = gnss_measurement_times;

[out, idx] = sort(gnss.times);
gnss.measurements = gnss.measurements(idx, :, :);
gnss.times = gnss.times(idx);

gnss_struct = gnss;

