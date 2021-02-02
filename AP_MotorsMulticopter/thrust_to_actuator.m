function actuator=thrust_to_actuator( thrust_in)
global spin_min
global spin_max
    thrust_in = constrain_value(thrust_in, 0.0, 1.0);
     actuator=spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);

end

