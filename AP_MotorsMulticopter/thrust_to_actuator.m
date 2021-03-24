function actuator=thrust_to_actuator( thrust_in)
% converts desired thrust to linearized actuator output in a range of 0~1
 global AP_Motors	

spin_min                = AP_Motors.spin_min;
spin_max                = AP_Motors.spin_max;

    thrust_in = constrain_value(thrust_in, 0.0, 1.0);
    actuator=spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);

AP_Motors.spin_min                = spin_min;
AP_Motors.spin_max                = spin_max;

end

