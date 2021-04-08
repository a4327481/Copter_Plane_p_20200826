function  input_euler_angle_roll_pitch_euler_rate_yaw(  euler_roll_angle_cd,   euler_pitch_angle_cd,   euler_yaw_rate_cds)
% Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing

global dt
global Test_w
global AC_Attitude

attitude_target_quat               = AC_Attitude.attitude_target_quat;
attitude_target_euler_angle        = AC_Attitude.attitude_target_euler_angle;
rate_bf_ff_enabled                 = AC_Attitude.rate_bf_ff_enabled;
accel_roll_max                     = AC_Attitude.accel_roll_max;
accel_pitch_max                    = AC_Attitude.accel_pitch_max;
accel_yaw_max                      = AC_Attitude.accel_yaw_max;
attitude_target_euler_rate         = AC_Attitude.attitude_target_euler_rate;
attitude_target_ang_vel            = AC_Attitude.attitude_target_ang_vel;
ang_vel_roll_max                   = AC_Attitude.ang_vel_roll_max;
ang_vel_pitch_max                  = AC_Attitude.ang_vel_pitch_max;
ang_vel_yaw_max                    = AC_Attitude.ang_vel_yaw_max;
input_tc                           = AC_Attitude.input_tc;




     euler_roll_angle_cd  = euler_roll_angle_cd  + Test_w.roll_target;
     euler_pitch_angle_cd = euler_pitch_angle_cd + Test_w.pitch_target; 
     euler_yaw_rate_cds   = euler_yaw_rate_cds   + Test_w.target_yaw_rate;
    % Convert from centidegrees on public interface to radians
      euler_roll_angle = radians(euler_roll_angle_cd * 0.01);
      euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01);
      euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01);

    % calculate the attitude target euler angles
    attitude_target_euler_angle=to_euler(attitude_target_quat);

%     Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
%     euler_roll_angle =euler_roll_angle+ get_roll_trim_rad();

    if (rate_bf_ff_enabled) %
        % translate the roll pitch and yaw acceleration limits to the euler axis
        euler_accel = euler_accel_limit(attitude_target_euler_angle, [radians(accel_roll_max * 0.01), radians(accel_pitch_max * 0.01), radians(accel_yaw_max * 0.01)]);

        % When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        % angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        % and an exponential decay specified by smoothing_gain at the end.
        attitude_target_euler_rate(1) = input_shaping_angle(wrap_PI(euler_roll_angle - attitude_target_euler_angle(1)), input_tc, euler_accel(1), attitude_target_euler_rate(1), dt);
        attitude_target_euler_rate(2) = input_shaping_angle(wrap_PI(euler_pitch_angle - attitude_target_euler_angle(2)), input_tc, euler_accel(2), attitude_target_euler_rate(2), dt);

        % When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        % the output rate towards the input rate.
        attitude_target_euler_rate(3) = input_shaping_ang_vel(attitude_target_euler_rate(3), euler_yaw_rate, euler_accel(3), dt);

        % Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        attitude_target_ang_vel=euler_rate_to_ang_vel(attitude_target_euler_angle, attitude_target_euler_rate );
        % Limit the angular velocity
        attitude_target_ang_vel=ang_vel_limit(attitude_target_ang_vel, radians(ang_vel_roll_max), radians(ang_vel_pitch_max), radians(ang_vel_yaw_max));
        % Convert body-frame angular velocity into euler angle derivative of desired attitude
        attitude_target_euler_rate=ang_vel_to_euler_rate(attitude_target_euler_angle, attitude_target_ang_vel);
      else %
        % When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        attitude_target_euler_angle(1)= euler_roll_angle;
        attitude_target_euler_angle(2) = euler_pitch_angle;
        attitude_target_euler_angle(3) =attitude_target_euler_angle(3)+ euler_yaw_rate * dt;
        % Compute quaternion target attitude
        attitude_target_quat=from_euler(attitude_target_euler_angle);

        % Set rate feedforward requests to zero
        attitude_target_euler_rate = [0.0 , 0.0 , 0.0 ];
        attitude_target_ang_vel = [0.0 , 0.0 , 0.0 ];
    end
 
AC_Attitude.attitude_target_quat               = attitude_target_quat;
AC_Attitude.attitude_target_euler_angle        = attitude_target_euler_angle;
AC_Attitude.rate_bf_ff_enabled                 = rate_bf_ff_enabled;
AC_Attitude.accel_roll_max                     = accel_roll_max;
AC_Attitude.accel_pitch_max                    = accel_pitch_max;
AC_Attitude.accel_yaw_max                      = accel_yaw_max;
AC_Attitude.attitude_target_euler_rate         = attitude_target_euler_rate;
AC_Attitude.attitude_target_ang_vel            = attitude_target_ang_vel;
AC_Attitude.ang_vel_roll_max                   = ang_vel_roll_max;
AC_Attitude.ang_vel_pitch_max                  = ang_vel_pitch_max;
AC_Attitude.ang_vel_yaw_max                    = ang_vel_yaw_max;
AC_Attitude.input_tc                           = input_tc; 
    % Call quaternion attitude controller
    attitude_controller_run_quat();
end

