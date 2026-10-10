within PhysicsAnalysis;
model Copter
  // Composition keeps the reusable plant and all attack logic in separate packages.
  parameter Real mass = 2.5644001;
  parameter Real ixx = 0.02601237985;
  parameter Real iyy = 0.02590943825;
  parameter Real izz = 0.045571756801;
  parameter Real Ct = 8.54858e-6;
  parameter Real Cm = 0.016;
  parameter Real Cn_r = -0.1;
  parameter Real pwm_min = 1100;
  parameter Real pwm_max = 1900;
  parameter Real omega_max = 1300;
  parameter Real lat0 = 40.414929;
  parameter Real lon0 = -86.932387;
  parameter Real ground_alt_wgs84 = 149;
  parameter Real earth_radius_m = 6378137;
  parameter Real attack_start_s = 140;
  parameter Real attack_duration_s = 60;
  parameter Real attack_ramp_s = 10;
  parameter Real magnetic_scale = 0;
  parameter Real wind_mean_mps = 0;
  parameter Real wind_gust_mps = 0;
  parameter Real wind_frequency_hz = 0.1;
  parameter Real wind_phase_rad = 0;
  parameter Real wind_from_deg = 270;
  parameter Real gps_north_m = 0;
  parameter Real gps_east_m = 0;
  parameter Real gps_up_m = 0;
  parameter Real gyro_bias_roll_dps = 0;
  parameter Real gyro_bias_pitch_dps = 0;
  parameter Real gyro_bias_yaw_dps = 0;
  parameter Real gyro_false_roll_dps = 0;
  parameter Real gyro_false_pitch_dps = 0;
  parameter Real gyro_false_yaw_dps = 0;
  parameter Real gyro_frequency_hz = 0.25;

  FastDyn.Copter vehicle(mass=mass, ixx=ixx, iyy=iyy, izz=izz, Ct=Ct, Cm=Cm,
    Cn_r=Cn_r, pwm_min=pwm_min, pwm_max=pwm_max, omega_max=omega_max,
    lat0=lat0, lon0=lon0, ground_alt_wgs84=ground_alt_wgs84, earth_radius_m=earth_radius_m,
    plant(external_force_b=wind_force_b));
  PhysicsAnalysis.AttackInputs attack(start_s=attack_start_s, duration_s=attack_duration_s,
    ramp_s=attack_ramp_s, magnetic_scale=magnetic_scale, wind_mean_mps=wind_mean_mps,
    wind_gust_mps=wind_gust_mps, wind_frequency_hz=wind_frequency_hz,
    wind_phase_rad=wind_phase_rad, wind_from_deg=wind_from_deg,
    gps_north_m=gps_north_m, gps_east_m=gps_east_m, gps_up_m=gps_up_m,
    gyro_bias_roll_dps=gyro_bias_roll_dps, gyro_bias_pitch_dps=gyro_bias_pitch_dps,
    gyro_bias_yaw_dps=gyro_bias_yaw_dps, gyro_false_roll_dps=gyro_false_roll_dps,
    gyro_false_pitch_dps=gyro_false_pitch_dps, gyro_false_yaw_dps=gyro_false_yaw_dps,
    gyro_frequency_hz=gyro_frequency_hz);
  input Real pwm[4](start={1000,1000,1000,1000});
  output Real accel[3];
  output Real gyro[3];
  output Real mag[3];
  output Real gps[3];
  output Real vel_ned[3];
  output Real yaw_deg "GPS yaw unavailable (MAVLink GPS_INPUT zero), not truth heading";
  output Real baro_altitude_m;
  output Real baro_pressure_pa;
  output Real baro_temperature_c;
  output Real baro_climb_rate_mps;
protected
  constant Real pi = 3.141592653589793;
  Real air_b[3];
  Real air_speed;
  Real ground_speed;
  Real wind_force_b[3];
equation
  vehicle.pwm = pwm;
  attack.motor_load = sum(vehicle.plant.omega_m .* vehicle.plant.omega_m)/(4*omega_max*omega_max);
  accel = vehicle.accel;
  gyro = vehicle.gyro + attack.gyro_bias_frd + attack.gyro_false_frd;
  mag = vehicle.mag + attack.mag_delta_frd;
  gps = vehicle.gps + {attack.gps_offset_neu[1]/earth_radius_m*180/pi,
    attack.gps_offset_neu[2]/(earth_radius_m*cos(lat0*pi/180))*180/pi, attack.gps_offset_neu[3]};
  vel_ned = vehicle.vel_ned + {attack.gps_velocity_neu[1],attack.gps_velocity_neu[2],-attack.gps_velocity_neu[3]};
  yaw_deg = 0;
  baro_altitude_m = vehicle.baro_altitude_m;
  baro_pressure_pa = vehicle.baro_pressure_pa;
  baro_temperature_c = vehicle.baro_temperature_c;
  baro_climb_rate_mps = vehicle.baro_climb_rate_mps;
  air_b = vehicle.plant.v_b - transpose(vehicle.plant.R)*attack.wind_nwu;
  air_speed = sqrt(sum(air_b .* air_b)+1e-12);
  ground_speed = sqrt(sum(vehicle.plant.v_b .* vehicle.plant.v_b)+1e-12);
  // Replace still-air drag by relative-air drag without double counting.
  wind_force_b = -0.5*vehicle.plant.rho*air_speed*(vehicle.drag_area .* air_b) - vehicle.linear_drag .* air_b
    + 0.5*vehicle.plant.rho*ground_speed*(vehicle.drag_area .* vehicle.plant.v_b) + vehicle.linear_drag .* vehicle.plant.v_b;
end Copter;
