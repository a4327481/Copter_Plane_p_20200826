function   AP_MotorsMulticopter_output_4a1()
global AP_Motors
global Copter_Plane

switch Copter_Plane.State
    case ENUM_State.E_Copter     
        AP_Motors.spool_desired = SpoolState.THROTTLE_UNLIMITED;
    case ENUM_State.E_Plane 
        AP_Motors.spool_desired = SpoolState.SHUT_DOWN;
        AP_Motors.spool_state = SpoolState.SHUT_DOWN;
end

update_throttle_mix();
% update throttle filter
update_throttle_filter();
% calc filtered battery voltage and lift_max
update_lift_max_from_batt_voltage();
% run spool logic
output_logic();
% calculate thrust
output_armed_stabilizing();
% apply any thrust compensation for the frame
%      thrust_compensation();

% convert rpy_thrust values to pwm
%     output_to_motors_4a1();
output_to_motors_g();
% output any booster throttle
%     output_boost_throttle();

% output raw roll/pitch/yaw/thrust
%     output_rpyt();
end

