within PhysicsAnalysis;
model AttackInputs
  parameter Real start_s = 140;
  parameter Real duration_s = 60;
  parameter Real ramp_s = 10;
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
  input Real motor_load;
  output Real gate;
  output Real model_current_a "Empirical motor-load proxy, NOT measured battery current";
  output Real mag_delta_frd[3] "Gauss";
  output Real wind_nwu[3] "Air velocity, m/s";
  output Real gps_offset_neu[3] "Spoof displacement, m";
  output Real gps_velocity_neu[3] "Time derivative of spoof displacement, m/s";
  output Real gyro_bias_frd[3] "rad/s";
  output Real gyro_false_frd[3] "rad/s";
protected
  constant Real pi = 3.141592653589793;
  Real u;
  Real v;
  Real rise;
  Real fall;
  Real rise_rate;
  Real fall_rate;
  Real gate_rate;
  Real speed;
equation
  // Smoothstep rise and fall; zero value AND velocity at both boundaries.
  u = min(1, max(0, (time-start_s)/ramp_s));
  v = min(1, max(0, (start_s+duration_s-time)/ramp_s));
  rise = u*u*(3-2*u);
  fall = v*v*(3-2*v);
  rise_rate = 6*u*(1-u)/ramp_s;
  fall_rate = -6*v*(1-v)/ramp_s;
  gate = rise*fall;
  gate_rate = rise_rate*fall + rise*fall_rate;
  model_current_a = max(0, 0.42 + 42.5908*motor_load);
  // Slopes adapted from the sibling study; illustrative, not a calibrated airframe.
  mag_delta_frd = gate*magnetic_scale*model_current_a*
    {0.004891567709167085, 0.004730743861609757, -0.0070414343670510615};
  speed = gate*(wind_mean_mps + wind_gust_mps*
    sin(2*pi*wind_frequency_hz*(time-start_s)+wind_phase_rad)^8);
  wind_nwu = {-speed*cos(wind_from_deg*pi/180), speed*sin(wind_from_deg*pi/180), 0};
  gps_offset_neu = gate*{gps_north_m,gps_east_m,gps_up_m};
  gps_velocity_neu = gate_rate*{gps_north_m,gps_east_m,gps_up_m};
  gyro_bias_frd = gate*pi/180*{gyro_bias_roll_dps,gyro_bias_pitch_dps,gyro_bias_yaw_dps};
  gyro_false_frd = gate*sin(2*pi*gyro_frequency_hz*(time-start_s))*pi/180*
    {gyro_false_roll_dps,gyro_false_pitch_dps,gyro_false_yaw_dps};
end AttackInputs;
