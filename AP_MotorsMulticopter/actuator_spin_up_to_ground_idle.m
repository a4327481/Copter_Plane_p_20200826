function     spin_ground_idle=actuator_spin_up_to_ground_idle() 
% gradually increase actuator output to spin_min

 global AP_Motors
 
 spin_up_ratio          = AP_Motors.spin_up_ratio;
 spin_min               = AP_Motors.spin_min;
 
      spin_ground_idle=constrain_value(spin_up_ratio, 0.0, 1.0) *spin_min ;

 AP_Motors.spin_up_ratio          = spin_up_ratio;
 AP_Motors.spin_min               = spin_min;

end

