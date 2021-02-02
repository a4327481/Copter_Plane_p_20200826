function     spin_ground_idle=actuator_spin_up_to_ground_idle() 
 global spin_up_ratio
 global spin_min
      spin_ground_idle=constrain_value(spin_up_ratio, 0.0, 1.0) *spin_min ;
end

