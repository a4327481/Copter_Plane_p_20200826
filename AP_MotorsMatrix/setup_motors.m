function setup_motors() 
    % MOTOR_FRAME_TYPE_H:
    % H frame set-up - same as X but motors spin in opposite directiSons
    global AP_Motors

    AP_MOTORS_MATRIX_YAW_FACTOR_CW         = AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CW;
    AP_MOTORS_MATRIX_YAW_FACTOR_CCW        = AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CCW;
    pitch_factor                           = AP_Motors.pitch_factor;
    Kx                                     = AP_Motors.Kx;
    
    add_motor(1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
    add_motor(2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW*2.5599);
    add_motor(3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
    add_motor(4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW*2.5599);
    pitch_factor=pitch_factor.*[1-Kx 1+Kx 1-Kx 1+Kx];
	
	    
	AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CW         = AP_MOTORS_MATRIX_YAW_FACTOR_CW;
    AP_Motors.AP_MOTORS_MATRIX_YAW_FACTOR_CCW        = AP_MOTORS_MATRIX_YAW_FACTOR_CCW;
    AP_Motors.pitch_factor                           = pitch_factor;
    AP_Motors.Kx                                     = Kx;
	
end

